#!/usr/bin/env python3
"""
Dynamic Mission Manager - Dinamik Görev Yöneticisi
====================================================

Sabit kanatlı İHA için tam otonom görev yönetim sistemi.

Özellikler:
    - Enum tabanlı State Machine (INIT → TAKEOFF → PATROL → RES_AREA → RTL → LAND)
    - Önceden tanımlı map waypointleri üzerinde döngüsel devriye
    - Anlık dinamik waypoint kuyruğu (deque) — öncelikli
    - Yasaklı alan (NFZ) kaçınması — güvenlik payı ile
    - Acil durum protokolü: kuyruk baskılama + en güvenli map noktasına kaçış
    - ROS 2 Topic'ler ile anlık komut alma (mock test kanalları)

Görev Akışı:
    INIT → WAIT → READY_ARM → TAKEOFF → PATROL ↔ RES_AREA → RTL → LAND

Mock Test Kanalları (Terminalden veri fırlatma):
    # Dinamik waypoint gönder (local metre koordinatı):
    ros2 topic pub --once /uav1/mock_dynamic_wp geometry_msgs/msg/Point "{x: 50.0, y: -20.0, z: 0.0}"

    # Yasaklı alan (NFZ) gönder (merkez x,y + yarıçap z):
    ros2 topic pub --once /uav1/mock_nfz geometry_msgs/msg/Point "{x: 100.0, y: 100.0, z: 20.0}"

Çalıştırma:
    # Terminal 1: MAVROS
    ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

    # Terminal 2: Dynamic Mission Manager
    cd ~/otopilot_ws && source install/setup.bash
    ros2 run iha_otopilot dynamic_mission
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
from enum import Enum
from collections import deque
import math


# ═══════════════════════════════════════════════════════════════
# DURUM TANIMLARI (STATE MACHINE)
# ═══════════════════════════════════════════════════════════════
class UavState(Enum):
    INIT = 0          # Başlat (Sistem kontrolleri)
    WAIT = 1          # Olumsuz → Bekle
    READY_ARM = 2     # Olumlu → ARM modu
    TAKEOFF = 3       # Kalkış
    PATROL = 4        # Ana map + dinamik waypoint takibi (OFFBOARD)
    RES_AREA = 5      # Yasaklı Bölge (NFZ) Acil Kaçış
    RTL = 6           # Return to Launch
    LAND = 7          # İniş


# ═══════════════════════════════════════════════════════════════
# MİSYON PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
TAKEOFF_ALT = 100.0           # Kalkış irtifası (m)
CRUISE_ALT = 100.0            # Seyir irtifası (m)
CRUISE_AIRSPEED = 18.0        # Normal seyir hızı (m/s)

# Waypoint kabul yarıçapı (sabit kanatlılar için büyük olmalı)
WAYPOINT_ACCEPT_RADIUS = 50.0

# NFZ güvenlik payı (metre) — yasaklı alan sınırına ek tampon
NFZ_SAFETY_BUFFER = 10.0

# ═══════════════════════════════════════════════════════════════
# ANA MAP WAYPOINT'LERİ (Local ENU metre — home noktasına göre)
# Uçak kalkış sonrası bu noktalarda döngüsel devriye yapacak
# ═══════════════════════════════════════════════════════════════
DEFAULT_MAP_WAYPOINTS = [
    (0.0, 200.0),
    (200.0, 200.0),
    (400.0, 200.0),
    (400.0, 0.0),
    (400.0, -200.0),
    (200.0, -200.0),
    (0.0, -200.0),
    (0.0, 0.0),
    (-200.0, 0.0),
    (-200.0, 200.0),
]


class DynamicMissionManager(Node):

    def __init__(self):
        super().__init__('dynamic_mission_manager_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ═══════════════════════════════════════════════════════════
        # SUBSCRIBERS — MAVROS
        # ═══════════════════════════════════════════════════════════
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.global_position_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.local_position_callback, qos_profile)
        self.home_sub = self.create_subscription(
            HomePosition, '/mavros/home_position/home',
            self.home_position_callback, qos_profile)

        # ═══════════════════════════════════════════════════════════
        # SUBSCRIBERS — MOCK TEST KANALLARI
        # ═══════════════════════════════════════════════════════════
        # Anlık dinamik waypoint (x, y = local metre)
        self.dyn_wp_sub = self.create_subscription(
            Point, '/uav1/mock_dynamic_wp', self.dynamic_wp_callback, 10)
        # Anlık yasaklı alan (x, y = merkez, z = yarıçap)
        self.nfz_sub = self.create_subscription(
            Point, '/uav1/mock_nfz', self.nfz_callback, 10)

        # ═══════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # ═══════════════════════════════════════════════════════════
        # SERVICE CLIENTS
        # ═══════════════════════════════════════════════════════════
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(
            ParamSetV2, '/mavros/param/set')

        # ═══════════════════════════════════════════════════════════
        # STATE DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════════
        self.mavros_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0
        self.log_counter = 0

        # ═══════════════════════════════════════════════════════════
        # STATE MACHINE
        # ═══════════════════════════════════════════════════════════
        self.current_state = UavState.INIT
        self.state_entry_done = False  # Duruma ilk girişte bir kez çalışan bayrak

        # ═══════════════════════════════════════════════════════════
        # HAFIZA: 3 KRİTİK VERİ YAPISI
        # ═══════════════════════════════════════════════════════════

        # 1) Ana Harita Waypointleri (önceden çizilmiş güvenli rota, döngüsel)
        self.map_waypoints = list(DEFAULT_MAP_WAYPOINTS)
        self.current_map_index = 0

        # 2) Dinamik Waypoint Kuyruğu (anlık gelen hedefler, FIFO)
        self.dynamic_queue = deque()

        # 3) Yasaklı Alanlar Listesi: [(x, y, radius), ...]
        self.nfz_list = []

        # O an MAVROS'a basılan nihai hedef
        self.current_target = None  # (x, y) tuple

        # ═══ Sürekli Setpoint Değişkenleri ═══
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = CRUISE_ALT
        self.enable_position_setpoint = False

        # ═══ PATROL Alt-Fazları ═══
        # 0 = OFFBOARD öncesi setpoint hazırlık (warmup)
        # 1 = OFFBOARD modu isteği gönderildi
        # 2 = Normal devriye
        self.patrol_phase = 0
        self.offboard_prep_counter = 0
        # PX4 OFFBOARD'u kabul etmesi için gereken minimum setpoint sayısı
        # 40 döngü × 0.1s (10 Hz timer) = ~4 saniye setpoint akışı
        self.OFFBOARD_PREP_THRESHOLD = 40

        # ═══ Hız Kontrol ═══
        self.current_target_speed = 0.0

        # ═══════════════════════════════════════════════════════════
        # TIMER'LAR
        # ═══════════════════════════════════════════════════════════
        # 10 Hz — Sürekli setpoint yayını (OFFBOARD stabilitesi)
        self.setpoint_timer = self.create_timer(0.1, self.timer_setpoint_callback)
        # 20 Hz — Ana kontrol döngüsü (State Machine + Güvenlik)
        self.control_timer = self.create_timer(0.05, self.main_control_loop)

        self.get_logger().info("═══ Dynamic Mission Manager Başlatıldı ═══")
        self.get_logger().info(f"    Map Waypoints: {len(self.map_waypoints)} nokta")
        self.get_logger().info(f"    Seyir İrtifası: {CRUISE_ALT}m")
        self.get_logger().info(f"    Waypoint Kabul Yarıçapı: {WAYPOINT_ACCEPT_RADIUS}m")

    # ═══════════════════════════════════════════════════════════════
    # TIMER: SÜREKLİ SETPOINT YAYINI (10 Hz)
    # ═══════════════════════════════════════════════════════════════
    def timer_setpoint_callback(self):
        """10 Hz'de arka planda sürekli setpoint gönderir (Offboard stabilitesi)."""
        if self.enable_position_setpoint:
            self.publish_setpoint(
                self.current_target_x,
                self.current_target_y,
                self.current_target_z)

    # ═══════════════════════════════════════════════════════════════
    # ANA KONTROL DÖNGÜSÜ (20 Hz)
    # ═══════════════════════════════════════════════════════════════
    def main_control_loop(self):
        """Ana kontrol döngüsü — State Machine + Güvenlik Denetimi."""

        # MAVROS origin hazır değilse hiçbir şey yapma
        if not self.home_pos or not self.mavros_origin_set:
            return

        # ─── 1. HER ZAMAN: SÜREKLİ GÜVENLİK KONTROLLERİ ───
        self.continuous_safety_checks()

        # ─── 2. STATE MACHINE İCRASI ───
        self.execute_state_machine()

    # ═══════════════════════════════════════════════════════════════
    # SÜREKLİ GÜVENLİK KONTROLLERİ
    # ═══════════════════════════════════════════════════════════════
    def continuous_safety_checks(self):
        """
        Diyagramın sol tarafındaki "Sürekli Kontrol Bloğu".
        Uçağın durumundan bağımsız, her döngüde çalışır.
        """
        # ─── PATROL'dayken: Mevcut hedef NFZ kontrolü ───
        if self.current_state == UavState.PATROL and self.current_target is not None:
            if not self.is_target_safe(self.current_target[0], self.current_target[1]):
                self.get_logger().warn(
                    "⚠ GÜVENLİK: Mevcut hedef yasaklı alanda! Rota yeniden hesaplanıyor...")
                self.handle_nfz_emergency()

    # ═══════════════════════════════════════════════════════════════
    # STATE MACHINE
    # ═══════════════════════════════════════════════════════════════
    def execute_state_machine(self):
        """Durum makinesini yürüt — her döngüde aktif durumun kodunu çalıştır."""

        # ==========================================
        # DURUM: INIT — Sistem kontrolleri
        # ==========================================
        if self.current_state == UavState.INIT:
            if not self.state_entry_done:
                self.get_logger().info("═══ INIT: Sistem kontrolleri yapılıyor... ═══")
                self.state_entry_done = True

            if self.check_systems_ready():
                self.get_logger().info("✓ Sistemler hazır → READY_ARM")
                self.transition_to(UavState.READY_ARM)
            else:
                self.transition_to(UavState.WAIT)

        # ==========================================
        # DURUM: WAIT — Sistem düzelene kadar bekle
        # ==========================================
        elif self.current_state == UavState.WAIT:
            if not self.state_entry_done:
                self.get_logger().info("═══ WAIT: Sistemler bekleniyor... ═══")
                self.state_entry_done = True

            if self.check_systems_ready():
                self.get_logger().info("✓ Sistemler düzeldi → READY_ARM")
                self.transition_to(UavState.READY_ARM)

        # ==========================================
        # DURUM: READY_ARM — ARM ve kalkış komutu
        # ==========================================
        elif self.current_state == UavState.READY_ARM:
            if not self.state_entry_done:
                self.get_logger().info(f"═══ READY_ARM: Kalkış hazırlığı ({TAKEOFF_ALT}m) ═══")
                self.send_takeoff_command(TAKEOFF_ALT)
                self.state_entry_done = True

            # AUTO.TAKEOFF moduna geçişi bekle, sonra ARM et
            if self.mavros_state.mode == "AUTO.TAKEOFF":
                self.get_logger().info(">> AUTO.TAKEOFF aktif — ARM ediliyor")
                self.send_arm_command(True)
                self.transition_to(UavState.TAKEOFF)

        # ==========================================
        # DURUM: TAKEOFF — İrtifa kazanımı
        # ==========================================
        elif self.current_state == UavState.TAKEOFF:
            if not self.state_entry_done:
                self.get_logger().info(f"═══ TAKEOFF: Hedef irtifa {TAKEOFF_ALT}m ═══")
                self.state_entry_done = True

            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   TAKEOFF | İrtifa: {self.current_pos['z']:.1f}m / {TAKEOFF_ALT}m"
                    f" | Setpoint: {'AKTIF' if self.enable_position_setpoint else 'KAPALI'}")
            self.log_counter += 1

            # ─── ERKEN SETPOINT BAŞLATMA ───
            # İrtifa hedefin 15m altına geldiğinde setpoint yayınını başlat.
            # PX4'ün OFFBOARD'u kabul etmesi için kalkış bitmeden ÖNCE
            # sürekli setpoint akışı hazır olmalı.
            if (self.current_pos['z'] >= (TAKEOFF_ALT - 15.0)
                    and not self.enable_position_setpoint):
                self.get_logger().info(
                    ">> TAKEOFF: Setpoint yayını erken başlatılıyor (OFFBOARD hazırlığı)")
                self.current_target_x = self.current_pos['x']
                self.current_target_y = self.current_pos['y']
                self.current_target_z = CRUISE_ALT
                self.enable_position_setpoint = True
                self.offboard_prep_counter = 0

            # Setpoint akışı sırasında sayacı artır
            if self.enable_position_setpoint:
                self.offboard_prep_counter += 1

            if self.current_pos['z'] >= (TAKEOFF_ALT - 5.0):
                self.get_logger().info("✓ Kalkış tamamlandı → PATROL")
                self.transition_to(UavState.PATROL)

        # ==========================================
        # DURUM: PATROL — Devriye (Waypoint takibi)
        # ==========================================
        elif self.current_state == UavState.PATROL:
            self.execute_patrol()

        # ==========================================
        # DURUM: RES_AREA — NFZ Acil Kaçış
        # ==========================================
        elif self.current_state == UavState.RES_AREA:
            self.execute_res_area()

        # ==========================================
        # DURUM: RTL — Eve dönüş
        # ==========================================
        elif self.current_state == UavState.RTL:
            if not self.state_entry_done:
                self.get_logger().info("═══ RTL: Eve dönülüyor ═══")
                self.enable_position_setpoint = False
                self.set_mode_command("AUTO.RTL")
                self.state_entry_done = True

            home_dist = self.distance_to(0.0, 0.0)
            if self.log_counter % 40 == 0:
                self.get_logger().info(
                    f"   RTL | Home: {home_dist:.0f}m | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if home_dist < 50.0 and self.current_pos['z'] < 80.0:
                self.get_logger().info("✓ RTL tamamlandı → LAND")
                self.transition_to(UavState.LAND)

        # ==========================================
        # DURUM: LAND — İniş
        # ==========================================
        elif self.current_state == UavState.LAND:
            if not self.state_entry_done:
                self.get_logger().info("═══ LAND: İniş yapılıyor ═══")
                self.set_mode_command("AUTO.LAND")
                self.state_entry_done = True

            if self.log_counter % 40 == 0:
                self.get_logger().info(
                    f"   LAND | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < 2.0:
                self.get_logger().info("═══ İNİŞ TAMAMLANDI ═══")

    # ═══════════════════════════════════════════════════════════════
    # PATROL DURUMU — Devriye + Dinamik Hedef Seçimi
    # ═══════════════════════════════════════════════════════════════
    def execute_patrol(self):
        """
        Devriye görevi (3 fazlı):
        
        Faz 0: PRE-OFFBOARD — PX4'e setpoint akışı gönder (warmup)
               PX4 OFFBOARD'u kabul etmeden önce sürekli setpoint görmeli.
               TAKEOFF'ta erken başlatılmışsa bu faz kısa sürer.
        Faz 1: OFFBOARD İSTEĞİ — Yeterli setpoint gönderildikten sonra
               OFFBOARD moduna geçiş isteği gönder.
        Faz 2: NORMAL DEVRİYE — Waypoint takibi, dinamik kuyruk, NFZ kontrolü.
        """

        # ═══════════════════════════════════════
        # FAZ 0: PRE-OFFBOARD SETPOINT HAZIRLIK
        # ═══════════════════════════════════════
        if self.patrol_phase == 0:
            if not self.state_entry_done:
                self.get_logger().info("═══ PATROL: Devriye başlatılıyor ═══")
                self.get_logger().info(f"    Map noktaları: {len(self.map_waypoints)}")
                self.get_logger().info(f"    Dinamik kuyruk: {len(self.dynamic_queue)}")
                self.get_logger().info(
                    f"    OFFBOARD hazırlık: {self.offboard_prep_counter}"
                    f"/{self.OFFBOARD_PREP_THRESHOLD} setpoint gönderildi")

                # İlk hedefi belirle
                self.determine_next_target()

                if self.current_target is not None:
                    self.current_target_x = self.current_target[0]
                    self.current_target_y = self.current_target[1]
                    self.current_target_z = CRUISE_ALT
                else:
                    # Hedef bulunamadıysa mevcut pozisyonu setpoint olarak bas
                    self.current_target_x = self.current_pos['x']
                    self.current_target_y = self.current_pos['y']
                    self.current_target_z = CRUISE_ALT

                # Setpoint yayınını başlat (henüz başlamamışsa)
                self.enable_position_setpoint = True
                self.state_entry_done = True

            # Sayacı artır (her 50ms'de bir — ana kontrol döngüsü hızı)
            self.offboard_prep_counter += 1

            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   PATROL [WARMUP] | Setpoint: {self.offboard_prep_counter}"
                    f"/{self.OFFBOARD_PREP_THRESHOLD}")
            self.log_counter += 1

            # Yeterli setpoint gönderildi → OFFBOARD isteğine geç
            if self.offboard_prep_counter >= self.OFFBOARD_PREP_THRESHOLD:
                self.get_logger().info(
                    ">> Yeterli setpoint gönderildi → OFFBOARD isteği gönderiliyor")
                self.patrol_phase = 1
                self.log_counter = 0
            return

        # ═══════════════════════════════════════
        # FAZ 1: OFFBOARD MOD İSTEĞİ
        # ═══════════════════════════════════════
        if self.patrol_phase == 1:
            self.set_mode_command("OFFBOARD")
            self.set_desired_speed(CRUISE_AIRSPEED)
            self.get_logger().info(">> OFFBOARD modu istendi → Normal devriyeye geçiliyor")
            self.patrol_phase = 2
            self.log_counter = 0
            return

        # ═══════════════════════════════════════
        # FAZ 2: NORMAL DEVRİYE
        # ═══════════════════════════════════════

        # ─── Hedef yoksa belirle ───
        if self.current_target is None:
            self.determine_next_target()
            if self.current_target is None:
                # Hiç güvenli hedef yok — yerinde kal
                if self.log_counter % 40 == 0:
                    self.get_logger().warn("⚠ TÜM HEDEFLER YASAKLI! Loiter bekleniyor...")
                self.log_counter += 1
                return

        # ─── Mevcut hedefe olan mesafe ───
        dist = self.distance_to(self.current_target[0], self.current_target[1])

        # ─── Setpoint güncelle ───
        self.current_target_x = self.current_target[0]
        self.current_target_y = self.current_target[1]
        self.current_target_z = CRUISE_ALT

        # ─── Loglama ───
        if self.log_counter % 20 == 0:
            nfz_count = len(self.nfz_list)
            queue_size = len(self.dynamic_queue)
            self.get_logger().info(
                f"   PATROL | Hedef: ({self.current_target[0]:.0f}, {self.current_target[1]:.0f}) "
                f"| Mesafe: {dist:.0f}m | Alt: {self.current_pos['z']:.0f}m "
                f"| Kuyruk: {queue_size} | NFZ: {nfz_count} "
                f"| Map: {self.current_map_index}/{len(self.map_waypoints)}")
        self.log_counter += 1

        # ─── Hedefe ulaşıldı mı? ───
        if dist < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info(
                f"✓ Hedefe ulaşıldı: ({self.current_target[0]:.0f}, {self.current_target[1]:.0f})")
            self.current_target = None
            self.determine_next_target()

            if self.current_target is not None:
                self.current_target_x = self.current_target[0]
                self.current_target_y = self.current_target[1]
                self.current_target_z = CRUISE_ALT
                self.get_logger().info(
                    f"→ Yeni hedef: ({self.current_target[0]:.0f}, {self.current_target[1]:.0f})")

    # ═══════════════════════════════════════════════════════════════
    # RES_AREA DURUMU — NFZ Acil Kaçış
    # ═══════════════════════════════════════════════════════════════
    def execute_res_area(self):
        """
        Yasaklı bölge acil kaçış:
        En güvenli map noktasına ulaşılınca PATROL'e dön.
        """
        if not self.state_entry_done:
            self.get_logger().warn("═══ RES_AREA: ACİL DURUM PROTOKOLÜ AKTİF ═══")
            self.state_entry_done = True

        if self.current_target is None:
            self.get_logger().error("RES_AREA: Güvenli hedef bulunamadı! Loiter modunda...")
            return

        dist = self.distance_to(self.current_target[0], self.current_target[1])

        # Setpoint güncelle
        self.current_target_x = self.current_target[0]
        self.current_target_y = self.current_target[1]
        self.current_target_z = CRUISE_ALT

        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"   RES_AREA | Kaçış hedefi: ({self.current_target[0]:.0f}, {self.current_target[1]:.0f}) "
                f"| Mesafe: {dist:.0f}m")
        self.log_counter += 1

        # Güvenli noktaya ulaşıldı → PATROL'e dön
        if dist < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info("✓ Güvenli noktaya ulaşıldı → PATROL'e dönülüyor")
            self.transition_to(UavState.PATROL)

    # ═══════════════════════════════════════════════════════════════
    # BEYNİN ÇEKİRDEĞİ: HEDEF SEÇME ALGORİTMASI
    # ═══════════════════════════════════════════════════════════════

    def determine_next_target(self):
        """
        Sıradaki hedefi belirle.
        
        Öncelik sırası:
            1. Dinamik kuyrukta güvenli nokta varsa → onu al
            2. Kuyruk boşsa/hepsi yasaklıysa → ana map'teki sıradaki güvenli nokta
        """
        # ─── 1. Öncelik: Dinamik kuyruk ───
        while len(self.dynamic_queue) > 0:
            candidate = self.dynamic_queue.popleft()
            if self.is_target_safe(candidate[0], candidate[1]):
                self.current_target = candidate
                self.get_logger().info(
                    f"  → Dinamik hedef seçildi: ({candidate[0]:.0f}, {candidate[1]:.0f})")
                return
            else:
                self.get_logger().info(
                    f"  ✖ Dinamik hedef yasaklı, es geçiliyor: ({candidate[0]:.0f}, {candidate[1]:.0f})")

        # ─── 2. Öncelik: Ana Map (döngüsel) ───
        if len(self.map_waypoints) == 0:
            self.get_logger().warn("⚠ Map waypoint listesi boş!")
            self.current_target = None
            return

        # Tüm map waypoint'lerini tara, güvenli olanı bul
        checked = 0
        while checked < len(self.map_waypoints):
            candidate = self.map_waypoints[self.current_map_index]
            self.current_map_index = (self.current_map_index + 1) % len(self.map_waypoints)
            checked += 1

            if self.is_target_safe(candidate[0], candidate[1]):
                self.current_target = candidate
                self.get_logger().info(
                    f"  → Map hedefi seçildi: ({candidate[0]:.0f}, {candidate[1]:.0f}) "
                    f"[{self.current_map_index}/{len(self.map_waypoints)}]")
                return

        # Hepsi yasaklıysa
        self.get_logger().error("⚠ TÜM MAP NOKTALARI YASAKLI!")
        self.current_target = None

    # ═══════════════════════════════════════════════════════════════
    # NFZ GÜVENLİK MATEMATİĞİ
    # ═══════════════════════════════════════════════════════════════

    def is_target_safe(self, target_x, target_y):
        """
        Bir noktanın tüm yasaklı alanlardan güvenli olup olmadığını kontrol et.
        
        Args:
            target_x: Hedef X (local ENU metre)
            target_y: Hedef Y (local ENU metre)
            
        Returns:
            True = Güvenli, False = Yasaklı alanda
        """
        for nfz_x, nfz_y, radius in self.nfz_list:
            distance = math.hypot(target_x - nfz_x, target_y - nfz_y)
            # Yasaklı alan + güvenlik payı
            if distance <= (radius + NFZ_SAFETY_BUFFER):
                return False
        return True

    def find_safest_map_point(self, current_uav_x, current_uav_y):
        """
        Uçağa en yakın ve güvenli olan map noktasını bul.
        
        Bu fonksiyon acil durumlarda çağrılır — uçağın hayatta kalma algoritması.
        
        Args:
            current_uav_x: Uçağın mevcut X pozisyonu
            current_uav_y: Uçağın mevcut Y pozisyonu
            
        Returns:
            (x, y) tuple veya None (hiç güvenli nokta yoksa)
        """
        safest_point = None
        min_distance = float('inf')

        for point in self.map_waypoints:
            p_x, p_y = point
            # Kural 1: Nokta yasaklı alanda olmamalı
            if self.is_target_safe(p_x, p_y):
                # Kural 2: Uçağa en yakın güvenli nokta olmalı
                dist = math.hypot(p_x - current_uav_x, p_y - current_uav_y)
                if dist < min_distance:
                    min_distance = dist
                    safest_point = point

        return safest_point

    def handle_nfz_emergency(self):
        """
        Acil Durum Protokolü:
        1. Dinamik kuyruğu tamamen temizle (hayatta kalmaya odaklan)
        2. En güvenli map noktasını bul
        3. RES_AREA durumuna geç
        """
        # Tüm dinamik hayalleri çöpe at
        queue_size = len(self.dynamic_queue)
        self.dynamic_queue.clear()
        if queue_size > 0:
            self.get_logger().warn(
                f"  🗑 Dinamik kuyruk temizlendi ({queue_size} hedef silindi)")

        # En güvenli map noktasını bul
        safest = self.find_safest_map_point(
            self.current_pos['x'], self.current_pos['y'])

        if safest:
            self.current_target = safest
            self.current_target_x = safest[0]
            self.current_target_y = safest[1]
            self.current_target_z = CRUISE_ALT
            self.get_logger().warn(
                f"  🛡 ACİL KAÇIŞ: En güvenli nokta → ({safest[0]:.0f}, {safest[1]:.0f})")
            self.transition_to(UavState.RES_AREA)
        else:
            self.get_logger().error(
                "  🔴 TÜM MAP YASAKLI! Mevcut konumda Loiter modunda kalınıyor.")
            # Mevcut pozisyonda kal
            self.current_target_x = self.current_pos['x']
            self.current_target_y = self.current_pos['y']
            self.current_target_z = CRUISE_ALT

    # ═══════════════════════════════════════════════════════════════
    # DURUM GEÇİŞ FONKSİYONU
    # ═══════════════════════════════════════════════════════════════
    def transition_to(self, new_state):
        """Durum geçişi — debug ve log kolaylığı."""
        old = self.current_state
        self.current_state = new_state
        self.state_entry_done = False
        self.log_counter = 0
        # PATROL'a her girişte fazı sıfırla
        if new_state == UavState.PATROL:
            self.patrol_phase = 0
        self.get_logger().info(f"  ⚡ DURUM GEÇİŞİ: {old.name} → {new_state.name}")

    # ═══════════════════════════════════════════════════════════════
    # SİSTEM KONTROL FONKSİYONU
    # ═══════════════════════════════════════════════════════════════
    def check_systems_ready(self):
        """
        Tüm sistemlerin uçuşa hazır olup olmadığını kontrol et.
        MAVROS bağlantısı + home position ayarlanmış olmalı.
        """
        return (self.mavros_state.connected and
                self.home_pos is not None and
                self.mavros_origin_set)

    # ═══════════════════════════════════════════════════════════════
    # MOCK TEST CALLBACK'LERİ
    # ═══════════════════════════════════════════════════════════════

    def dynamic_wp_callback(self, msg):
        """
        Terminalden gelen dinamik waypoint'i kuyruğa ekle.
        msg.x, msg.y = local ENU metre koordinatı
        msg.z = kullanılmıyor (sabit irtifa)
        """
        self.dynamic_queue.append((msg.x, msg.y))
        self.get_logger().info(
            f"  📡 YENİ DİNAMİK HEDEF ALINDI → ({msg.x:.0f}, {msg.y:.0f}) "
            f"| Kuyruk boyutu: {len(self.dynamic_queue)}")

    def nfz_callback(self, msg):
        """
        Terminalden gelen yasaklı alan bilgisini listeye ekle.
        msg.x, msg.y = NFZ merkez koordinatı (local ENU metre)
        msg.z = yarıçap (metre)
        """
        nfz_x = msg.x
        nfz_y = msg.y
        radius = msg.z

        self.nfz_list.append((nfz_x, nfz_y, radius))
        self.get_logger().warn(
            f"  🔴 ACİL! YENİ YASAKLI ALAN → Merkez: ({nfz_x:.0f}, {nfz_y:.0f}), "
            f"Yarıçap: {radius:.0f}m | Toplam NFZ: {len(self.nfz_list)}")

        # ─── ANLIK TEHLİKE DEĞERLENDİRMESİ ───
        # PATROL'dayken ve mevcut hedefe etki ediyorsa → acil müdahale
        if self.current_state == UavState.PATROL and self.current_target is not None:
            if not self.is_target_safe(self.current_target[0], self.current_target[1]):
                self.get_logger().warn(
                    "  ⚠ Mevcut hedef yeni yasaklı alandan etkilendi!")
                self.handle_nfz_emergency()

    # ═══════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ═══════════════════════════════════════════════════════════════

    def distance_to(self, x, y):
        """Uçağın mevcut konumundan (x, y) noktasına 2D mesafe."""
        return math.hypot(x - self.current_pos['x'], y - self.current_pos['y'])

    def global_to_local(self, lat, lon):
        """GPS koordinatlarını local ENU'ya çevir (Home noktasına göre)."""
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean)
        y = d_lat * R
        return x, y

    # ═══════════════════════════════════════════════════════════════
    # PUBLISH FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def publish_setpoint(self, x, y, z):
        """Local position setpoint gönder."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.local_pos_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    # KOMUT FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def send_takeoff_command(self, relative_alt):
        """Takeoff komutu (göreceli irtifa)."""
        req = CommandTOL.Request()
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.takeoff_client.call_async(req)

    def send_arm_command(self, arm_status):
        """ARM/DISARM komutu."""
        req = CommandBool.Request()
        req.value = arm_status
        self.arming_client.call_async(req)

    def set_mode_command(self, mode):
        """Mod değiştirme komutu."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def set_desired_speed(self, target_speed):
        """
        PX4'ün seyir hızını ayarla.
        FW_AIRSPD_MAX ve FW_AIRSPD_TRIM parametrelerini dinamik olarak değiştirir.
        """
        if self.current_target_speed == target_speed:
            return

        is_slowing_down = target_speed < self.current_target_speed
        self.current_target_speed = target_speed

        trim_speed = target_speed - 1.0

        # PX4 aerodinamik kuralı: Yavaşlıyorsan önce TRIM düşür
        if is_slowing_down or target_speed == 0.0:
            self._set_param("FW_AIRSPD_TRIM", trim_speed)
            self._set_param("FW_AIRSPD_MAX", target_speed)
        else:
            self._set_param("FW_AIRSPD_MAX", target_speed)
            self._set_param("FW_AIRSPD_TRIM", trim_speed)

        self.get_logger().info(
            f"  ⚡ HIZ: Seyir hızı {target_speed} m/s olarak ayarlandı")

    def _set_param(self, param_id, value):
        """Tek bir PX4 parametresini ayarla."""
        req = ParamSetV2.Request()
        req.param_id = param_id
        req.value = ParameterValue()
        req.value.type = ParameterType.PARAMETER_DOUBLE
        req.value.double_value = float(value)
        self.param_set_client.call_async(req)

    # ═══════════════════════════════════════════════════════════════
    # MAVROS CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def state_callback(self, msg):
        self.mavros_state = msg

    def local_position_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z

    def global_position_callback(self, msg):
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(
                f"HOME: {msg.latitude:.6f}, {msg.longitude:.6f}")
            self.get_logger().info(
                f"HOME AMSL: {self.home_alt_amsl:.1f}m")

    def home_position_callback(self, msg):
        if not self.mavros_origin_set and msg.geo.latitude != 0:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS ORIGIN SET")


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = DynamicMissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
