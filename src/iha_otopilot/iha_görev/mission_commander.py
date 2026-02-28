#!/usr/bin/env python3
"""
Mission Commander Node — Karar Katmanı (HFSM Beyin)
=====================================================

Tüm algı verilerini toplar, State Machine ile durum yönetimi yapar ve
taktiksel emirleri trajectory_generator'a iletir.

State Machine:
    INIT → WAIT → READY_ARM → TAKEOFF → PATROL ↔ TRACKING ↔ RES_AREA → SEARCHING → RTL → LAND

Dinlediği Topic'ler:
    /gorev/telemetry_status    → TelemetryStatus (Algı)
    /gorev/target_info         → TargetInfo (Gözler)
    /mavros/state              → MAVROS bağlantı durumu
    /mavros/home_position/home → Home position
    /mavros/global_position/global → İlk GPS fix
    /uav1/mock_dynamic_wp      → Dinamik waypoint (Point: x=lat, y=lon)
    /uav1/mock_nfz             → Yasaklı alan (Point: x=lat, y=lon, z=yarıçap)

Yayınladığı Topic:
    /gorev/mission_command     → MissionCommand (→ Yörünge)

Mock Test Komutları (Terminalden):
    # Dinamik waypoint gönder (lat, lon):
    ros2 topic pub --once /uav1/mock_dynamic_wp geometry_msgs/msg/Point "{x: 39.820, y: 32.750, z: 0.0}"

    # Yasaklı alan gönder (lat, lon, yarıçap_metre):
    ros2 topic pub --once /uav1/mock_nfz geometry_msgs/msg/Point "{x: 39.821, y: 32.751, z: 50.0}"


package.xml dosyasında  iha_messages eklendi bu sayede TelemetryStatus, TargetInfo, MissionCommand mesajlarını kullanabiliyoruz.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from iha_messages.msg import TelemetryStatus, TargetInfo, MissionCommand
from enum import Enum
from collections import deque
# deque (Double-ended Queue - Çift Uçlu Kuyruk)
# deque (okunuşu: "dek"), uçağın kısa süreli hafızası gibidir.
# Ne işe yarar? İçine belli sayıda veri alabilen ve kapasitesi dolunca en eski veriyi otomatik olarak "çöpe atan" akıllı bir listedir.
# Listeye 11. eleman geldiğinde, 1. eleman (en eski olan) otomatik olarak silinir.
# Böylece hafıza her zaman taze ve sabit boyutta kalır.
import math



class UavState(Enum):
    INIT = 0
    WAIT = 1
    READY_ARM = 2
    TAKEOFF = 3
    PATROL = 4        # Ana map + dinamik waypoint takibi
    TRACKING = 5      # Hedef takip
    RES_AREA = 6      # NFZ acil kaçış
    SEARCHING = 7     # Hedef arama patterni
    RTL = 8
    LAND = 9


# ═══════════════════════════════════════════════════════════════
# MİSYON PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
TAKEOFF_ALT = 100.0
CRUISE_ALT = 100.0
CRUISE_SPEED = 18.0
WAYPOINT_ACCEPT_RADIUS = 50.0
NFZ_SAFETY_BUFFER = 10.0

# Hedef takip
TRACKING_TIMEOUT = 3.0   # Hedef kayıp toleransı (s)

# Ana Map Waypointleri (LAT, LON) — Yarışma sahası koordinatları
# Home position alındıktan sonra otomatik Local ENU'ya çevrilir
DEFAULT_MAP_WAYPOINTS_GPS = [
    (39.820500, 32.750000),
    (39.822000, 32.750000),
    (39.823500, 32.750000),
    (39.823500, 32.752500),
    (39.823500, 32.755000),
    (39.822000, 32.755000),
    (39.820500, 32.755000),
    (39.820500, 32.752500),
]


class MissionCommander(Node):

    def __init__(self):
        super().__init__('mission_commander_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        # ═══════════════════════════════════════════════════════════
        # SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════
        # Algı katmanından
        self.telemetry_sub = self.create_subscription(
            TelemetryStatus, '/gorev/telemetry_status',
            self.telemetry_callback, 10)
        self.target_sub = self.create_subscription(
            TargetInfo, '/gorev/target_info',
            self.target_callback, 10)

        # MAVROS'tan doğrudan (durum, pozisyon, home)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.local_pos_callback, qos_profile)
        self.home_sub = self.create_subscription(
            HomePosition, '/mavros/home_position/home',
            self.home_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.global_pos_callback, qos_profile)

        # Mock kanallar
        self.dyn_wp_sub = self.create_subscription(
            Point, '/uav1/mock_dynamic_wp', self.dynamic_wp_callback, 10)
        self.nfz_sub = self.create_subscription(
            Point, '/uav1/mock_nfz', self.nfz_callback, 10)

        # ═══════════════════════════════════════════════════════════
        # PUBLISHER
        # ═══════════════════════════════════════════════════════════
        self.command_pub = self.create_publisher(
            MissionCommand, '/gorev/mission_command', 10)

        # ═══════════════════════════════════════════════════════════
        # SERVICE CLIENTS (ARM, TAKEOFF, MODE)
        # ═══════════════════════════════════════════════════════════
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # ═══════════════════════════════════════════════════════════
        # STATE MACHINE
        # ═══════════════════════════════════════════════════════════
        self.current_state = UavState.INIT
        self.state_entry_done = False
        self.log_counter = 0

        # PATROL alt-fazları (OFFBOARD warmup)
        self.patrol_phase = 0
        self.offboard_prep_counter = 0
        self.OFFBOARD_PREP_THRESHOLD = 40

        # ═════════════════════════════════════════════════════════
        # HAFIZA
        # ═════════════════════════════════════════════════════════
        self.map_waypoints_gps = list(DEFAULT_MAP_WAYPOINTS_GPS)
        self.map_waypoints = []  # Home gelince GPS→Local çevrilecek
        self.map_waypoints_converted = False
        self.current_map_index = 0
        self.dynamic_queue = deque()

        # Neden Burada maxlen Yok?
        # Kodun içindeki dynamic_queue, uçağın gitmesi gereken **"Dinamik Görev Noktaları"**nı tutan bir sıradır (kuyruktur).
        # Eğer maxlen=10 yapsaydık: Sen uçağa terminalden peş peşe 12 tane hedef nokta gönderdiğinde, uçak 11. noktayı alınca en baştaki 1. noktayı hafızasından otomatik olarak silerdi. Bu da uçağın gitmesi gereken bir noktayı "unutmasına" neden olurdu.
        # Şu anki haliyle: Bu bir görev kuyruğu olduğu için, uçağa ne kadar nokta verirsek verelim hepsini sırayla ziyaret etmesini istiyoruz. Uçak bir noktaya vardığında popleft() komutuyla o noktayı kuyruktan çıkarıyor (yani hafızasını kendi temizliyor).

        self.nfz_list = []       # [(local_x, local_y, radius), ...]
        self.nfz_gps_list = []   # [(lat, lon, radius), ...] — loglama için
        self.current_target = None  # (x, y) tuple

        # ═══════════════════════════════════════════════════════════
        # TELEMETRİ CACHE
        # ═══════════════════════════════════════════════════════════
        self.mavros_state = State()
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # ═══════════════════════════════════════════════════════════
        # HEDEF TAKİP
        # ═══════════════════════════════════════════════════════════
        self.target_info = None        # Son gelen TargetInfo
        self.target_detected = False
        self.locked_target_id = None   # Kilitlenen Hedef ID (None ise boştayız)
        self.tracking_start_time = 0.0
        
        # Telemetri durumu (Algı katmanından gelen temiz veri)
        self.airspeed = 0.0
        self.is_connected = False
        self.is_critical_telemetry = False

        self.return_state_after_tracking = UavState.PATROL

        # ═══════════════════════════════════════════════════════════
        # TIMER: 20 Hz Karar Döngüsü
        # ═══════════════════════════════════════════════════════════
        self.timer = self.create_timer(0.05, self.decision_loop)

        self.get_logger().info("═══ Mission Commander (Beyin) başlatıldı ═══")
        self.get_logger().info(f"    Map Waypoints: {len(self.map_waypoints)}")

    # ═══════════════════════════════════════════════════════════════
    # ANA KARAR DÖNGÜSÜ (20 Hz)
    # ═══════════════════════════════════════════════════════════════
    def decision_loop(self):
        """Beynin ana düşünce döngüsü."""
        if not self.home_pos or not self.mavros_origin_set:
            return

        # Sürekli NFZ güvenlik taraması
        self.nfz_safety_scan()

        # State Machine
        self.execute_state_machine()

    # ═══════════════════════════════════════════════════════════════
    # NFZ GÜVENLİK TARAMASI
    # ═══════════════════════════════════════════════════════════════
    def nfz_safety_scan(self):
        """PATROL'dayken mevcut hedefi NFZ'ye karşı sürekli tara."""
        if self.current_state == UavState.PATROL and self.current_target is not None:
            if not self.is_target_safe(self.current_target[0], self.current_target[1]):
                self.get_logger().warn(
                    "⚠ GÜVENLİK: Mevcut hedef yasaklı alanda!")
                self.handle_nfz_emergency()

    # ═══════════════════════════════════════════════════════════════
    # STATE MACHINE
    # ═══════════════════════════════════════════════════════════════
    def execute_state_machine(self):

        # ── INIT ──
        if self.current_state == UavState.INIT:
            if not self.state_entry_done:
                self.get_logger().info("═══ INIT: Sistemler kontrol ediliyor... ═══")
                self.state_entry_done = True
            if self.check_systems_ready():
                self.transition_to(UavState.READY_ARM)
            else:
                self.transition_to(UavState.WAIT)

        # ── WAIT ──
        elif self.current_state == UavState.WAIT:
            if not self.state_entry_done:
                self.get_logger().info("═══ WAIT: Sistemler bekleniyor... ═══")
                self.state_entry_done = True
            if self.check_systems_ready():
                self.transition_to(UavState.READY_ARM)

        # ── READY_ARM ──
        elif self.current_state == UavState.READY_ARM:
            if not self.state_entry_done:
                self.get_logger().info(f"═══ READY_ARM: Kalkış ({TAKEOFF_ALT}m) ═══")
                self.send_takeoff_command(TAKEOFF_ALT)
                self.state_entry_done = True
            if self.mavros_state.mode == "AUTO.TAKEOFF":
                self.get_logger().info(">> ARM ediliyor")
                self.send_arm_command(True)
                self.transition_to(UavState.TAKEOFF)

        # ── TAKEOFF ──
        elif self.current_state == UavState.TAKEOFF:
            if not self.state_entry_done:
                self.get_logger().info(f"═══ TAKEOFF: Hedef {TAKEOFF_ALT}m ═══")
                self.offboard_prep_counter = 0
                self.state_entry_done = True

            # ─── HER DÖNGÜDE GOTO komutu yayınla ───
            # trajectory_generator bu komutu alınca setpoint yayınına
            # başlayacak → PX4 sürekli setpoint akışı görecek
            self.publish_command("GOTO", self.pos_x, self.pos_y, CRUISE_ALT,
                                 CRUISE_SPEED, "TAKEOFF_PREP")
            self.offboard_prep_counter += 1

            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   TAKEOFF | Alt: {self.pos_z:.1f}m / {TAKEOFF_ALT}m"
                    f" | Setpoint: {self.offboard_prep_counter}")
            self.log_counter += 1

            if self.pos_z >= (TAKEOFF_ALT - 5.0):
                self.get_logger().info(
                    f"✓ Kalkış OK → PATROL (setpoint: {self.offboard_prep_counter})")
                self.transition_to(UavState.PATROL)

        # ── PATROL ──
        elif self.current_state == UavState.PATROL:
            self.execute_patrol()

        # ── TRACKING ──
        elif self.current_state == UavState.TRACKING:
            self.execute_tracking()

        # ── RES_AREA ──
        elif self.current_state == UavState.RES_AREA:
            self.execute_res_area()

        # ── SEARCHING ──
        elif self.current_state == UavState.SEARCHING:
            self.execute_searching()

        # ── RTL ──
        elif self.current_state == UavState.RTL:
            if not self.state_entry_done:
                self.get_logger().info("═══ RTL ═══")
                self.set_mode_command("AUTO.RTL")
                self.state_entry_done = True
            home_dist = self.distance_to(0.0, 0.0)
            if home_dist < 50.0 and self.pos_z < 80.0:
                self.transition_to(UavState.LAND)

        # ── LAND ──
        elif self.current_state == UavState.LAND:
            if not self.state_entry_done:
                self.get_logger().info("═══ LAND ═══")
                self.set_mode_command("AUTO.LAND")
                self.state_entry_done = True

    # ═══════════════════════════════════════════════════════════════
    # PATROL (3 Fazlı)
    # ═══════════════════════════════════════════════════════════════
    def execute_patrol(self):

        # ── FAZ 0: OFFBOARD Warmup ──
        if self.patrol_phase == 0:
            if not self.state_entry_done:
                self.get_logger().info("═══ PATROL: Warmup başlatılıyor ═══")
                self.determine_next_target()
                self.state_entry_done = True

            tx = self.current_target[0] if self.current_target else self.pos_x
            ty = self.current_target[1] if self.current_target else self.pos_y
            self.publish_command("GOTO", tx, ty, CRUISE_ALT, CRUISE_SPEED, "WARMUP")
            self.offboard_prep_counter += 1

            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   PATROL [WARMUP] | {self.offboard_prep_counter}"
                    f"/{self.OFFBOARD_PREP_THRESHOLD}")
            self.log_counter += 1

            if self.offboard_prep_counter >= self.OFFBOARD_PREP_THRESHOLD:
                self.patrol_phase = 1
                self.log_counter = 0
            return

        # ── FAZ 1: OFFBOARD Mod İsteği ──
        if self.patrol_phase == 1:
            self.set_mode_command("OFFBOARD")
            self.get_logger().info(">> OFFBOARD modu istendi")
            self.patrol_phase = 2
            self.log_counter = 0
            return

        # ── FAZ 2: Normal Devriye ──
        if self.current_target is None:
            self.determine_next_target()
            if self.current_target is None:
                if self.log_counter % 40 == 0:
                    self.get_logger().warn("⚠ TÜM HEDEFLER YASAKLI!")
                self.log_counter += 1
                return

        dist = self.distance_to(self.current_target[0], self.current_target[1])

        # Taktiksel emir yayınla
        self.publish_command("GOTO", self.current_target[0], self.current_target[1],
                             CRUISE_ALT, CRUISE_SPEED, "PATROL")

        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"   PATROL | ({self.current_target[0]:.0f}, {self.current_target[1]:.0f}) "
                f"| {dist:.0f}m | Kuyruk: {len(self.dynamic_queue)} "
                f"| NFZ: {len(self.nfz_list)}")
            self.log_nfz_distances()
        self.log_counter += 1

        # Hedef tespit → TRACKING'e geç
        if self.target_detected and self.target_info is not None:
            self.get_logger().info("  🎯 HEDEF! → TRACKING")
            self.return_state_after_tracking = UavState.PATROL
            self.transition_to(UavState.TRACKING)
            return

        # Hedefe ulaşıldı
        if dist < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info(
                f"✓ Ulaşıldı: ({self.current_target[0]:.0f}, {self.current_target[1]:.0f})")
            self.current_target = None
            self.determine_next_target()

    # ═══════════════════════════════════════════════════════════════
    # TRACKING (Hedef Takip)
    # ═══════════════════════════════════════════════════════════════
    def execute_tracking(self):
        if not self.state_entry_done:
            self.get_logger().info("═══ TRACKING: Hedef takibi ═══")
            self.tracking_start_time = self.get_clock().now().nanoseconds / 1e9
            self.state_entry_done = True

        if self.target_detected and self.target_info is not None:
            ti = self.target_info
            self.publish_command("TRACK", ti.target_x, ti.target_y, ti.target_z,
                                 CRUISE_SPEED, "TRACKING", is_dynamic=True)
            if self.log_counter % 10 == 0:
                self.get_logger().info(
                    f"   TRACKING | ({ti.target_x:.0f}, {ti.target_y:.0f}) "
                    f"| Güven: {ti.confidence:.2f}")
            self.log_counter += 1
        else:
            # Hedef kayboldu → SEARCHING'e geç
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.tracking_start_time > TRACKING_TIMEOUT:
                self.get_logger().warn("  ⚠ Hedef kayıp → SEARCHING")
                self.transition_to(UavState.SEARCHING)

    # ═══════════════════════════════════════════════════════════════
    # SEARCHING (Hedef Arama)
    # ═══════════════════════════════════════════════════════════════
    def execute_searching(self):
        """Hedef kaybolduğunda 8 çizme patterni — şimdilik mevcut pozisyonda orbit."""
        if not self.state_entry_done:
            self.get_logger().info("═══ SEARCHING: Hedef aranıyor ═══")
            self.search_start_time = self.get_clock().now().nanoseconds / 1e9
            self.state_entry_done = True

        # Mevcut konumda orbit et
        self.publish_command("ORBIT", self.pos_x, self.pos_y, CRUISE_ALT,
                             CRUISE_SPEED, "SEARCHING")

        # Hedef tekrar tespit edildi → TRACKING'e dön
        if self.target_detected and self.target_info is not None:
            self.get_logger().info("  🎯 Hedef tekrar bulundu! → TRACKING")
            self.transition_to(UavState.TRACKING)
            return

        # 30 saniye aradıktan sonra PATROL'e dön
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.search_start_time > 30.0:
            self.get_logger().info("  ✖ Arama zaman aşımı → PATROL'e dönülüyor")
            self.transition_to(self.return_state_after_tracking)

        if self.log_counter % 40 == 0:
            elapsed = now - self.search_start_time
            self.get_logger().info(f"   SEARCHING | {elapsed:.0f}s / 30s")
        self.log_counter += 1

    # ═══════════════════════════════════════════════════════════════
    # RES_AREA (NFZ Kaçış)
    # ═══════════════════════════════════════════════════════════════
    def execute_res_area(self):
        if not self.state_entry_done:
            self.get_logger().warn("═══ RES_AREA: ACİL KAÇIŞ ═══")
            self.state_entry_done = True

        if self.current_target is None:
            self.get_logger().error("Güvenli hedef yok!")
            return

        dist = self.distance_to(self.current_target[0], self.current_target[1])
        self.publish_command("GOTO", self.current_target[0], self.current_target[1],
                             CRUISE_ALT, CRUISE_SPEED, "RES_AREA")

        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"   RES_AREA | Kaçış → ({self.current_target[0]:.0f}, "
                f"{self.current_target[1]:.0f}) | {dist:.0f}m")
        self.log_counter += 1

        if dist < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info("✓ Güvenli nokta → PATROL")
            self.transition_to(UavState.PATROL)

    # ═══════════════════════════════════════════════════════════════
    # HEDEF SEÇME ALGORİTMASI
    # ═══════════════════════════════════════════════════════════════
    def determine_next_target(self):
        """Öncelik: dinamik kuyruk → ana map."""
        while len(self.dynamic_queue) > 0:
            candidate = self.dynamic_queue.popleft()
            if self.is_target_safe(candidate[0], candidate[1]):
                self.current_target = candidate
                self.get_logger().info(
                    f"  → Dinamik: ({candidate[0]:.0f}, {candidate[1]:.0f})")
                return
        # Ana map
        if len(self.map_waypoints) == 0:
            self.current_target = None
            return
        checked = 0
        while checked < len(self.map_waypoints):
            candidate = self.map_waypoints[self.current_map_index]
            self.current_map_index = (self.current_map_index + 1) % len(self.map_waypoints)
            checked += 1
            if self.is_target_safe(candidate[0], candidate[1]):
                self.current_target = candidate
                return
        self.current_target = None

    # ═══════════════════════════════════════════════════════════════
    # NFZ MATEMATİĞİ
    # ═══════════════════════════════════════════════════════════════
    def is_target_safe(self, x, y):
        for nfz_x, nfz_y, radius in self.nfz_list:
            if math.hypot(x - nfz_x, y - nfz_y) <= (radius + NFZ_SAFETY_BUFFER):
                return False
        return True

    def find_safest_map_point(self):
        safest = None
        min_d = float('inf')
        for p in self.map_waypoints:
            if self.is_target_safe(p[0], p[1]):
                d = math.hypot(p[0] - self.pos_x, p[1] - self.pos_y)
                if d < min_d:
                    min_d = d
                    safest = p
        return safest

    def handle_nfz_emergency(self):
        q_size = len(self.dynamic_queue)
        self.dynamic_queue.clear()
        if q_size > 0:
            self.get_logger().warn(f"  🗑 Kuyruk temizlendi ({q_size})")
        safest = self.find_safest_map_point()
        if safest:
            self.current_target = safest
            self.get_logger().warn(
                f"  🛡 Kaçış → ({safest[0]:.0f}, {safest[1]:.0f})")
            self.transition_to(UavState.RES_AREA)
        else:
            self.get_logger().error("  🔴 TÜM MAP YASAKLI!")

    # ═══════════════════════════════════════════════════════════════
    # YARDIMCI
    # ═══════════════════════════════════════════════════════════════
    def transition_to(self, new_state):
        old = self.current_state
        self.current_state = new_state
        self.state_entry_done = False
        self.log_counter = 0

        # Takip/Arama bittiyse kilidi de sıfırla
        if old in [UavState.TRACKING, UavState.SEARCHING] and \
           new_state not in [UavState.TRACKING, UavState.SEARCHING]:
            self.locked_target_id = None
            self.get_logger().info("  🔓 Hedef Kilidi Kaldırıldı")

        if new_state == UavState.PATROL:
            self.patrol_phase = 0
        self.get_logger().info(f"  ⚡ {old.name} → {new_state.name}")

    def check_systems_ready(self):
        return (self.mavros_state.connected and
                self.home_pos is not None and
                self.mavros_origin_set)

    def distance_to(self, x, y):
        return math.hypot(x - self.pos_x, y - self.pos_y)

    def gps_to_local(self, lat, lon):
        """GPS (lat, lon) → Local ENU (x, y) metre çevirici."""
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean)  # East
        y = d_lat * R                        # North
        return x, y

    def log_nfz_distances(self):
        """Ucağın tüm NFZ alanlarına olan mesafesini logla."""
        if len(self.nfz_list) == 0:
            return
        for i, (nfz_x, nfz_y, radius) in enumerate(self.nfz_list):
            dist = math.hypot(self.pos_x - nfz_x, self.pos_y - nfz_y)
            margin = dist - radius  # Negatifse alanın içinde
            gps_lat, gps_lon, _ = self.nfz_gps_list[i]
            if margin < 0:
                self.get_logger().error(
                    f"   🔴 NFZ-{i+1} | İÇİNDE! {abs(margin):.0f}m | "
                    f"({gps_lat:.6f}, {gps_lon:.6f}) R={radius:.0f}m")
            elif margin < 100:
                self.get_logger().warn(
                    f"   ⚠ NFZ-{i+1} | {margin:.0f}m uzakta | "
                    f"({gps_lat:.6f}, {gps_lon:.6f}) R={radius:.0f}m")
            else:
                self.get_logger().info(
                    f"   ✅ NFZ-{i+1} | {margin:.0f}m uzakta | "
                    f"({gps_lat:.6f}, {gps_lon:.6f}) R={radius:.0f}m")

    def publish_command(self, cmd_type, x, y, z, speed, state_name, is_dynamic=False):
        msg = MissionCommand()
        msg.command_type = cmd_type
        msg.target_x = float(x)
        msg.target_y = float(y)
        msg.target_z = float(z)
        msg.speed = float(speed)
        msg.state_name = state_name
        msg.is_dynamic_target = is_dynamic
        self.command_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    # KOMUT FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════
    def send_takeoff_command(self, relative_alt):
        req = CommandTOL.Request()
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.takeoff_client.call_async(req)

    def send_arm_command(self, arm):
        req = CommandBool.Request()
        req.value = arm
        self.arming_client.call_async(req)

    def set_mode_command(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════
    def telemetry_callback(self, msg):
        """Telemetri monitöründen gelen temiz raporu işle."""
        self.airspeed = msg.airspeed
        self.is_connected = msg.is_connected
        
        # Kritik durum geçişi kontrolü (Sadece yeni bir alarm geldiğinde uyar)
        if msg.is_critical and not self.is_critical_telemetry:
            self.get_logger().warn(f"  🚨 KRİTİK DURUM BİLDİRİMİ: {msg.warning_message}")
        
        self.is_critical_telemetry = msg.is_critical

    def local_pos_callback(self, msg):
        """Doğrudan MAVROS'tan pozisyon — gecikme yok."""
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        self.pos_z = msg.pose.position.z

    def target_callback(self, msg):
        """Hedef Takip Kilitleme Mantığı."""
        # 1. Hiç kilit yoksa ve yeni hedef geldiyse: KİLİTLEN
        if self.locked_target_id is None:
            if msg.detected:
                self.locked_target_id = msg.target_id
                self.target_info = msg
                self.target_detected = True
                self.tracking_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().warn(f"  🎯 HEDEFE KİLİTLENDİ! ID: {msg.target_id}")
        
        # 2. Zaten birine kilitliysek: Sadece onu dinle
        elif msg.target_id == self.locked_target_id:
            self.target_info = msg
            self.target_detected = msg.detected
            if msg.detected:
                self.tracking_start_time = self.get_clock().now().nanoseconds / 1e9
            else:
                # Kilitli hedef 'detected=False' attıysa (timeout vision_processor'da olur)
                pass 
        
        # 3. Başka bir ID geldiyse: Görmezden gel (Loglama çok seyrek)
        else:
            if self.log_counter % 100 == 0:
                 self.get_logger().info(f"  ℹ Farklı hedef pas geçildi (Kilit: {self.locked_target_id}, Gelen: {msg.target_id})")

    def state_callback(self, msg):
        self.mavros_state = msg

    def home_callback(self, msg):
        if not self.mavros_origin_set and msg.geo.latitude != 0:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS ORIGIN SET")

    def global_pos_callback(self, msg):
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(
                f"HOME: {msg.latitude:.6f}, {msg.longitude:.6f} "
                f"| AMSL: {self.home_alt_amsl:.1f}m")

            # ─── Map waypointlerini GPS → Local çevir ───
            if not self.map_waypoints_converted:
                self.map_waypoints = []
                for lat, lon in self.map_waypoints_gps:
                    x, y = self.gps_to_local(lat, lon)
                    self.map_waypoints.append((x, y))
                self.map_waypoints_converted = True
                self.get_logger().info(
                    f"   ✅ {len(self.map_waypoints)} map waypoint GPS→Local çevrildi")
                for i, (lat, lon) in enumerate(self.map_waypoints_gps):
                    lx, ly = self.map_waypoints[i]
                    self.get_logger().info(
                        f"      WP-{i+1}: ({lat:.6f}, {lon:.6f}) → ({lx:.0f}, {ly:.0f})m")

    def dynamic_wp_callback(self, msg):
        """msg.x=lat, msg.y=lon → GPS→Local çevir ve kuyruğa ekle."""
        if not self.home_pos:
            self.get_logger().warn("  ⚠ Home yok, waypoint es geçildi")
            return
        local_x, local_y = self.gps_to_local(msg.x, msg.y)
        self.dynamic_queue.append((local_x, local_y))
        self.get_logger().info(
            f"  📡 Dinamik WP → GPS: ({msg.x:.6f}, {msg.y:.6f}) "
            f"→ Local: ({local_x:.0f}, {local_y:.0f})m "
            f"| Kuyruk: {len(self.dynamic_queue)}")

    def nfz_callback(self, msg):
        """msg.x=lat, msg.y=lon, msg.z=yarıçap(m) → GPS→Local."""
        if not self.home_pos:
            self.get_logger().warn("  ⚠ Home yok, NFZ es geçildi")
            return
        local_x, local_y = self.gps_to_local(msg.x, msg.y)
        radius = msg.z
        self.nfz_list.append((local_x, local_y, radius))
        self.nfz_gps_list.append((msg.x, msg.y, radius))

        # Ucağın bu yeni NFZ'ye mesafesi
        dist = math.hypot(self.pos_x - local_x, self.pos_y - local_y)
        margin = dist - radius

        self.get_logger().warn(
            f"  🔴 YENİ NFZ-{len(self.nfz_list)} → GPS: ({msg.x:.6f}, {msg.y:.6f}) "
            f"R={radius:.0f}m | Local: ({local_x:.0f}, {local_y:.0f})m "
            f"| Uçak mesafe: {margin:.0f}m")

        # Anlık tehlike kontrolü
        if self.current_state == UavState.PATROL and self.current_target is not None:
            if not self.is_target_safe(self.current_target[0], self.current_target[1]):
                self.handle_nfz_emergency()


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
