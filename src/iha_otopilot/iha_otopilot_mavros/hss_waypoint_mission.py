#!/usr/bin/env python3
"""
HSS Waypoint Mission - Dinamik Waypoint Takibi + HSS Yasaklı Bölge Kaçınma
============================================================================

Görev Akışı:
    TAKEOFF → PATROL (waypoint takibi) → (HSS_AVOID) → LAND

Özellikler:
    - Dinamik waypoint sözlüğü (runtime'da değiştirilebilir)
    - HSS yasaklı bölge yönetimi (10s gecikme, 60s aktif)
    - Point-in-Circle kontrol (uçak + hedef waypoint kontrolü)
    - Manuel müdahale: waypoint ekleme/silme, HSS aktive etme, LAND komutu

Operasyon Alanı (Köşe Koordinatları):
    1: 40.230761, 29.001866
    2: 40.229318, 29.009345
    3: 40.233739, 29.009292
    4: 40.233848, 28.998916

Çalıştırma:
    # Terminal 1: MAVROS
    source /opt/ros/humble/setup.bash
    ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

    # Terminal 2: HSS Mission
    cd ~/otopilot_ws && source install/setup.bash
    ros2 run iha_otopilot hss_mission

Operatör Müdahale API (main() içindeki timer'lar ile):
    node.add_waypoint(key, lat, lon, alt, name)   → Waypoint ekle/güncelle
    node.remove_waypoint(key)                      → Waypoint sil
    node.activate_hss(zone_id, lat, lon, radius)   → HSS bölge aktive et
    node.deactivate_hss(zone_id)                   → HSS bölge kapat
    node.command_land()                            → İniş komutu
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, HomePosition, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from vision_msgs.msg import Detection2DArray
import math
import time


# ═══════════════════════════════════════════════════════════════
# OPERASYON ALANI KÖŞELERİ
# ═══════════════════════════════════════════════════════════════
OPERATION_AREA = [
    (40.230761, 29.001866),  # Köşe 1
    (40.229318, 29.009345),  # Köşe 2
    (40.233739, 29.009292),  # Köşe 3
    (40.233848, 28.998916),  # Köşe 4
]

# ═══════════════════════════════════════════════════════════════
# MİSYON PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
TAKEOFF_ALT = 100.0          # Kalkış irtifası (m)
PATROL_ALT = 100.0           # Devriye irtifası (m)
WAYPOINT_RADIUS = 50.0       # Waypoint'e ulaşıldı sayılma mesafesi (m)
PATROL_SPEED = 18.0          # Devriye hızı (m/s)

# HSS Parametreleri
HSS_ACTIVATION_DELAY = 10.0  # HSS aktive olduktan sonra efektif olma süresi (s)
HSS_ACTIVE_DURATION = 60.0   # HSS aktif kalma süresi (s)
HSS_SAFETY_BUFFER = 50.0     # HSS bölge sınırına ek güvenlik mesafesi (m)

# Hedef Takip Parametreleri
TRACKING_LOCK_DURATION = 4.0   # Kilitlenme süresi (saniye)
TRACKING_CENTER_TOL = 0.20     # Bounding box merkez toleransı (%20)
CAMERA_WIDTH = 640             # Kamera çözünürlüğü (piksel)
CAMERA_HEIGHT = 480
TRACKING_YAW_KP = 0.3         # Yaw PID oransal kazanç
TRACKING_YAW_KD = 0.05        # Yaw PID türevsel kazanç
TRACKING_PITCH_KP = 0.2       # Pitch PID oransal kazanç
TRACKING_SPEED = 20.0          # Takip hızı (m/s)


class HSSWaypointMission(Node):

    def __init__(self):
        super().__init__('hss_waypoint_mission_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ═══════════════════════════════════════════════════════════
        # SUBSCRIBERS
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

        # YOLO Tespit Subscriber
        self.yolo_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections',
            self.yolo_detection_callback, 10)

        # Rakip İHA GPS Subscriber (simülasyonda: /uav2/mavros/...)
        self.target_gps_sub = self.create_subscription(
            NavSatFix, '/target/global_position',
            self.target_gps_callback, qos_profile)

        # ═══════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.attitude_pub = self.create_publisher(
            AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # ═══════════════════════════════════════════════════════════
        # SERVICE CLIENTS
        # ═══════════════════════════════════════════════════════════
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # ═══════════════════════════════════════════════════════════
        # STATE DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════════
        self.current_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gps = None
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0

        # ═══════════════════════════════════════════════════════════
        # MİSYON DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════════
        self.mission_queue = []
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0

        # ═══════════════════════════════════════════════════════════
        # DİNAMİK WAYPOINT SÖZLÜĞÜ
        # ═══════════════════════════════════════════════════════════
        # Operatör tarafından anlık olarak değiştirilebilir
        # Format: {key: {'lat': float, 'lon': float, 'alt': float, 'name': str}}
        self.active_waypoints = {}
        self.waypoint_keys_order = []   # Sıralama için anahtar listesi
        self.current_wp_index = 0       # Şu anki waypoint indeksi
        self.waypoints_local = {}       # Local koordinatlara çevrilmiş hali

        # Başlangıç waypointleri (operatör değiştirebilir)
        self._init_default_waypoints()

        # ═══════════════════════════════════════════════════════════
        # HSS YASAKLI BÖLGE YÖNETİCİSİ
        # ═══════════════════════════════════════════════════════════
        # Format: {zone_id: {
        #   'lat': float, 'lon': float, 'radius': float,
        #   'command_time': float,    # Komut gönderilme zamanı
        #   'active_time': float,     # Efektif olma zamanı (command_time + 10s)
        #   'expire_time': float,     # Kapanma zamanı (active_time + 60s)
        #   'state': str              # 'pending' | 'active' | 'expired'
        # }}
        self.hss_zones = {}

        # ═══════════════════════════════════════════════════════════
        # HEDEF TAKİP DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════════
        self.target_detected = False
        self.target_bbox = None           # {'x': norm_cx, 'y': norm_cy, 'w': norm_w, 'h': norm_h}
        self.target_gps = None            # {'lat': float, 'lon': float, 'alt': float}
        self.target_lost_time = None      # Hedef kaybedilme zamanı
        self.tracking_start_time = None   # 4s kilitlenme timer başlangıcı
        self.tracking_success_count = 0   # Başarılı kilitlenme sayısı
        self.prev_yaw_error = 0.0         # PID türevsel bileşen
        self.tracking_enabled = True      # Takip özelliği açık/kapalı

        # ═══════════════════════════════════════════════════════════
        # LAND KOMUTU DEĞİŞKENİ
        # ═══════════════════════════════════════════════════════════
        self.land_commanded = False

        # ═══════════════════════════════════════════════════════════
        # TIMER (50ms = 20Hz kontrol döngüsü)
        # ═══════════════════════════════════════════════════════════
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("═══ HSS Waypoint Mission Node Başlatıldı ═══")

    # ═══════════════════════════════════════════════════════════════
    # BAŞLANGIÇ WAYPOINT'LERİ
    # ═══════════════════════════════════════════════════════════════
    def _init_default_waypoints(self):
        """Varsayılan başlangıç waypointleri. Operatör bunları değiştirebilir."""
        self.add_waypoint(0, 40.230761, 29.001866, PATROL_ALT, "WP1_Kose1")
        self.add_waypoint(1, 40.229318, 29.009345, PATROL_ALT, "WP2_Kose2")
        self.add_waypoint(2, 40.233739, 29.009292, PATROL_ALT, "WP3_Kose3")
        self.add_waypoint(3, 40.233848, 28.998916, PATROL_ALT, "WP4_Kose4")

    # ═══════════════════════════════════════════════════════════════
    # OPERATÖR MÜDAHALESİ FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def add_waypoint(self, key, lat, lon, alt, name=None):
        """
        Waypoint sözlüğüne yeni waypoint ekle veya mevcut olanı güncelle.
        
        Args:
            key: Waypoint benzersiz anahtarı (int veya str)
            lat: Enlem (derece)
            lon: Boylam (derece)
            alt: İrtifa (metre, göreceli)
            name: İsim (opsiyonel)
        """
        wp_name = name or f"WP_{key}"
        self.active_waypoints[key] = {
            'lat': lat, 'lon': lon, 'alt': alt, 'name': wp_name
        }
        # Sıralama listesini güncelle
        if key not in self.waypoint_keys_order:
            self.waypoint_keys_order.append(key)
        self.get_logger().info(
            f"  ✚ Waypoint eklendi/güncellendi: {wp_name} "
            f"({lat:.6f}, {lon:.6f}, {alt:.0f}m) [key={key}]")

    def remove_waypoint(self, key):
        """
        Waypoint sözlüğünden waypoint sil.
        
        Args:
            key: Silinecek waypoint anahtarı
        """
        if key in self.active_waypoints:
            wp = self.active_waypoints.pop(key)
            if key in self.waypoint_keys_order:
                self.waypoint_keys_order.remove(key)
            self.get_logger().info(f"  ✖ Waypoint silindi: {wp['name']} [key={key}]")
            # İndeksi düzelt
            if self.current_wp_index >= len(self.waypoint_keys_order):
                self.current_wp_index = 0
        else:
            self.get_logger().warn(f"  ⚠ Waypoint bulunamadı: key={key}")

    def activate_hss(self, zone_id, lat, lon, radius):
        """
        HSS yasaklı bölge aktive et. 10 saniye gecikme ile etkin olacak.
        
        Args:
            zone_id: Bölge benzersiz kimliği
            lat: Bölge merkez enlemi
            lon: Bölge merkez boylamı
            radius: Bölge yarıçapı (metre)
        """
        now = time.time()
        self.hss_zones[zone_id] = {
            'lat': lat,
            'lon': lon,
            'radius': radius,
            'command_time': now,
            'active_time': now + HSS_ACTIVATION_DELAY,
            'expire_time': now + HSS_ACTIVATION_DELAY + HSS_ACTIVE_DURATION,
            'state': 'pending'
        }
        self.get_logger().info(
            f"  ⚡ HSS AKTİVE EDİLDİ: zone={zone_id} | "
            f"Merkez=({lat:.6f}, {lon:.6f}) | Yarıçap={radius:.0f}m")
        self.get_logger().info(
            f"     → {HSS_ACTIVATION_DELAY:.0f}s sonra aktif olacak, "
            f"{HSS_ACTIVE_DURATION:.0f}s açık kalacak")

    def deactivate_hss(self, zone_id):
        """
        HSS yasaklı bölgeyi manuel olarak deaktive et.
        
        Args:
            zone_id: Deaktive edilecek bölge kimliği
        """
        if zone_id in self.hss_zones:
            self.hss_zones[zone_id]['state'] = 'expired'
            self.get_logger().info(f"  ✖ HSS DEAKTİVE: zone={zone_id}")
        else:
            self.get_logger().warn(f"  ⚠ HSS bölgesi bulunamadı: zone={zone_id}")

    def command_land(self):
        """LAND komutu gönder. Uçak mevcut waypoint takibini bırakıp home'a iner."""
        self.land_commanded = True
        self.get_logger().info("  🛬 LAND KOMUTU ALINDI - İniş başlatılıyor...")

    def start_tracking(self):
        """Manuel olarak hedef takip modunu aktif et."""
        self.tracking_enabled = True
        self.get_logger().info("  🎯 HEDEF TAKİP AKTİF")

    def stop_tracking(self):
        """Hedef takip modunu devre dışı bırak."""
        self.tracking_enabled = False
        self.target_detected = False
        self.target_bbox = None
        self.get_logger().info("  ✖ HEDEF TAKİP DEVRE DIŞI")

    # ═══════════════════════════════════════════════════════════════
    # HSS BÖLGE YÖNETİMİ
    # ═══════════════════════════════════════════════════════════════

    def update_hss_zones(self):
        """
        HSS bölgelerinin durumunu zamanlayıcılara göre güncelle.
        Her kontrol döngüsünde çağrılır.
        """
        now = time.time()
        for zone_id, zone in self.hss_zones.items():
            if zone['state'] == 'pending':
                if now >= zone['active_time']:
                    zone['state'] = 'active'
                    remaining = zone['expire_time'] - now
                    self.get_logger().info(
                        f"  🔴 HSS AKTİF: zone={zone_id} | "
                        f"{remaining:.0f}s kaldı")
            elif zone['state'] == 'active':
                if now >= zone['expire_time']:
                    zone['state'] = 'expired'
                    self.get_logger().info(
                        f"  🟢 HSS KAPANDI: zone={zone_id} | "
                        f"Bölge artık güvenli")

    def get_active_hss_zones(self):
        """Aktif HSS bölgelerinin listesini döndür."""
        return {
            zid: z for zid, z in self.hss_zones.items()
            if z['state'] == 'active'
        }

    def is_point_in_hss(self, lat, lon):
        """
        Bir GPS noktasının aktif HSS bölgesi içinde olup olmadığını kontrol et.
        
        Args:
            lat: Kontrol edilecek enlem
            lon: Kontrol edilecek boylam
            
        Returns:
            (bool, zone_id or None): Bölge içindeyse True ve zone_id
        """
        for zone_id, zone in self.get_active_hss_zones().items():
            dist = self.gps_distance(lat, lon, zone['lat'], zone['lon'])
            if dist < (zone['radius'] + HSS_SAFETY_BUFFER):
                return True, zone_id
        return False, None

    def is_local_point_in_hss(self, x, y):
        """
        Local ENU koordinatındaki bir noktanın aktif HSS bölgesi içinde 
        olup olmadığını kontrol et.
        
        Args:
            x: Local X (East)
            y: Local Y (North)
            
        Returns:
            (bool, zone_id or None): Bölge içindeyse True ve zone_id
        """
        for zone_id, zone in self.get_active_hss_zones().items():
            zone_x, zone_y = self.global_to_local(zone['lat'], zone['lon'])
            dist = math.sqrt((x - zone_x)**2 + (y - zone_y)**2)
            if dist < (zone['radius'] + HSS_SAFETY_BUFFER):
                return True, zone_id
        return False, None

    def find_safe_waypoint_index(self):
        """
        HSS bölgesi dışında olan ilk waypoint indeksini bul.
        Mevcut indeksten başlayarak tüm listeyi tarar.
        
        Returns:
            int or None: Güvenli waypoint indeksi, bulunamazsa None
        """
        n = len(self.waypoint_keys_order)
        if n == 0:
            return None

        for i in range(n):
            idx = (self.current_wp_index + i) % n
            key = self.waypoint_keys_order[idx]
            wp = self.active_waypoints[key]
            in_hss, _ = self.is_point_in_hss(wp['lat'], wp['lon'])
            if not in_hss:
                return idx
        return None  # Tüm waypointler yasaklı bölgede

    # ═══════════════════════════════════════════════════════════════
    # MİSYON SETUP
    # ═══════════════════════════════════════════════════════════════

    def setup_mission(self):
        """Misyon kurulumu - waypointleri local koordinatlara çevir."""
        if not self.home_pos:
            return

        self.get_logger().info("═══ MİSYON KURULUMU ═══")

        # Waypoint sözlüğünü local koordinatlara çevir
        self._update_local_waypoints()

        # Waypoint listesini logla
        for key in self.waypoint_keys_order:
            wp = self.active_waypoints[key]
            lw = self.waypoints_local[key]
            self.get_logger().info(
                f"  {wp['name']}: GPS({wp['lat']:.6f}, {wp['lon']:.6f}) "
                f"→ Local({lw['x']:.0f}, {lw['y']:.0f}, {wp['alt']:.0f}m)")

        # Misyon kuyruğu
        self.mission_queue.append({'type': 'TAKEOFF', 'alt': TAKEOFF_ALT})
        self.mission_queue.append({'type': 'PATROL'})

    def _update_local_waypoints(self):
        """Waypoint sözlüğünü local ENU koordinatlarına çevir."""
        self.waypoints_local = {}
        for key, wp in self.active_waypoints.items():
            x, y = self.global_to_local(wp['lat'], wp['lon'])
            self.waypoints_local[key] = {'x': x, 'y': y, 'alt': wp['alt']}

    # ═══════════════════════════════════════════════════════════════
    # ANA KONTROL DÖNGÜSÜ
    # ═══════════════════════════════════════════════════════════════

    def control_loop(self):
        """50ms'de bir çalışan ana kontrol döngüsü."""
        if not self.home_pos or not self.mavros_origin_set:
            return

        # ═══ HSS BÖLGE GÜNCELLEME (her döngüde) ═══
        self.update_hss_zones()

        # ═══ LAND KOMUTU KONTROLÜ ═══
        if self.land_commanded:
            # Mevcut görevi iptal et, LAND'e geç
            if self.current_task is None or self.current_task['type'] != 'LAND':
                self.current_task = {'type': 'LAND'}
                self.task_state = 0
                self.log_counter = 0
                self.mission_queue.clear()

        # ═══ İLK ÇALIŞMADA MİSYONU KUR ═══
        if (len(self.mission_queue) == 0 and self.current_task is None
                and self.task_state == 0 and not self.land_commanded):
            self.setup_mission()

        # ═══ YENİ GÖREVE GEÇ ═══
        if self.current_task is None:
            if len(self.mission_queue) > 0:
                self.current_task = self.mission_queue.pop(0)
                self.task_state = 0
                self.log_counter = 0
                self.get_logger().info(
                    f"\n>> YENİ GÖREV: {self.current_task['type']}")
            else:
                if self.task_state != 999:
                    self.get_logger().info("═══ TÜM GÖREVLER TAMAMLANDI ═══")
                    self.task_state = 999
                return

        # ═══ GÖREV YÖNLENDİRME ═══
        task_type = self.current_task['type']

        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'PATROL':
            self.execute_patrol()
        elif task_type == 'TRACKING':
            self.execute_tracking()
        elif task_type == 'LAND':
            self.execute_land()

    # ═══════════════════════════════════════════════════════════════
    # GÖREV FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def execute_takeoff(self):
        """Kalkış görevi - AUTO.TAKEOFF modu ve ARM."""
        target_alt = self.current_task['alt']

        if self.task_state == 0:
            self.get_logger().info(f">> TAKEOFF: Hedef irtifa {target_alt}m")
            self.send_takeoff_command(target_alt)
            self.task_state = 1

        elif self.task_state == 1:
            if self.current_state.mode == "AUTO.TAKEOFF":
                self.get_logger().info(">> AUTO.TAKEOFF aktif - ARM ediliyor")
                self.send_arm_command(True)
                self.task_state = 2

        elif self.task_state == 2:
            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   TAKEOFF | İrtifa: {self.current_pos['z']:.0f}m "
                    f"/ {target_alt}m")
            self.log_counter += 1

            if self.current_pos['z'] > (target_alt - 5.0):
                self.get_logger().info(">> TAKEOFF TAMAMLANDI")
                self.finish_task()

    def execute_patrol(self):
        """
        Waypoint devriye görevi.
        - Dinamik waypoint sözlüğünden sıradaki hedefe gider
        - HSS yasaklı bölge kontrolü yapar
        - Hedef waypoint yasaklı alandaysa sonraki güvenli waypoint'e geçer
        """
        if len(self.waypoint_keys_order) == 0:
            self.get_logger().warn("  ⚠ Waypoint sözlüğü boş!")
            return

        # ═══ OFFBOARD MODUNA GEÇ (ilk çağrı) ═══
        if self.task_state == 0:
            self.get_logger().info(">> PATROL: OFFBOARD moduna geçiliyor")
            # Local waypoint'leri güncelle
            self._update_local_waypoints()
            # Mevcut konumu ilk setpoint olarak gönder
            self.publish_setpoint(
                self.current_pos['x'],
                self.current_pos['y'],
                self.current_pos['z'])
            self.set_mode_command("OFFBOARD")
            self.task_state = 1
            return

        # ═══ WAYPOINT TAKİBİ ═══
        # Sözlük runtime'da değişebilir, her döngüde kontrol et
        if len(self.waypoint_keys_order) == 0:
            return

        # İndeks sınır kontrolü
        if self.current_wp_index >= len(self.waypoint_keys_order):
            self.current_wp_index = 0

        # Mevcut hedef waypoint
        current_key = self.waypoint_keys_order[self.current_wp_index]
        if current_key not in self.active_waypoints:
            # Waypoint silindi, indeksi düzelt
            self.current_wp_index = 0
            if len(self.waypoint_keys_order) == 0:
                return
            current_key = self.waypoint_keys_order[self.current_wp_index]

        wp = self.active_waypoints[current_key]

        # ═══ HSS KONTROLÜ ═══
        # Hedef waypoint yasaklı bölgede mi?
        in_hss, hss_zone_id = self.is_point_in_hss(wp['lat'], wp['lon'])
        if in_hss:
            safe_idx = self.find_safe_waypoint_index()
            if safe_idx is not None and safe_idx != self.current_wp_index:
                old_name = wp['name']
                self.current_wp_index = safe_idx
                current_key = self.waypoint_keys_order[self.current_wp_index]
                wp = self.active_waypoints[current_key]
                self.get_logger().info(
                    f"  ⚠ HSS KAÇINMA: {old_name} yasaklı bölgede (zone={hss_zone_id}) "
                    f"→ {wp['name']} hedefine geçildi")
            else:
                # Tüm waypointler yasaklı bölgede → loiter (mevcut konumda bekle)
                if self.log_counter % 40 == 0:
                    self.get_logger().warn(
                        "  ⚠ TÜM WAYPOINT'LER YASAKLI BÖLGEDE - Loiter modunda bekleniyor")
                self.publish_setpoint(
                    self.current_pos['x'],
                    self.current_pos['y'],
                    PATROL_ALT)
                self.log_counter += 1
                return

        # Uçağın kendisi yasaklı bölgede mi? (acil kaçış)
        aircraft_in_hss, aircraft_hss_zone = self.is_local_point_in_hss(
            self.current_pos['x'], self.current_pos['y'])
        if aircraft_in_hss:
            if self.log_counter % 20 == 0:
                self.get_logger().warn(
                    f"  🔴 UÇAK YASAKLI BÖLGEDE! (zone={aircraft_hss_zone}) "
                    f"→ Güvenli waypoint'e kaçılıyor")

        # ═══ Local waypoint'leri güncelle (sözlük değişmiş olabilir) ═══
        if current_key not in self.waypoints_local:
            self._update_local_waypoints()

        lw = self.waypoints_local.get(current_key)
        if lw is None:
            self._update_local_waypoints()
            lw = self.waypoints_local.get(current_key)
            if lw is None:
                return

        wp_x, wp_y, wp_alt = lw['x'], lw['y'], lw['alt']

        # Waypoint'e git
        dist_to_wp = self.distance_to(wp_x, wp_y)
        self.publish_position_with_velocity(
            wp_x, wp_y, wp_alt,
            max_speed=PATROL_SPEED,
            target_x=wp_x, target_y=wp_y)

        # ═══ LOGLAMA ═══
        if self.log_counter % 20 == 0:
            active_hss_count = len(self.get_active_hss_zones())
            pending_hss = sum(
                1 for z in self.hss_zones.values() if z['state'] == 'pending')
            hss_status = ""
            if active_hss_count > 0:
                hss_status = f" | 🔴 HSS: {active_hss_count} aktif"
            elif pending_hss > 0:
                hss_status = f" | ⏳ HSS: {pending_hss} beklemede"

            self.get_logger().info(
                f"   PATROL → {wp['name']} | Mesafe: {dist_to_wp:.0f}m "
                f"| Alt: {self.current_pos['z']:.0f}m"
                f"| WP: {self.current_wp_index+1}/{len(self.waypoint_keys_order)}"
                f"{hss_status}")
        self.log_counter += 1

        # ═══ HEDEF TESPİT KONTROLÜ (YOLO) ═══
        if self.tracking_enabled and self.target_detected and self.target_bbox:
            self.get_logger().info(
                "  🎯 HEDEF TESPİT EDİLDİ! → TRACKING moduna geçiliyor")
            self.mission_queue.insert(0, {'type': 'PATROL'})  # Tracking sonrası PATROL'e dön
            self.current_task = {'type': 'TRACKING'}
            self.task_state = 0
            self.log_counter = 0
            return

        # ═══ WAYPOINT'E ULAŞILDI MI? ═══
        if dist_to_wp < WAYPOINT_RADIUS:
            self.get_logger().info(f"  ✓ {wp['name']} ULAŞILDI!")
            # Sonraki waypoint'e geç (döngüsel)
            self.current_wp_index = (
                (self.current_wp_index + 1) % len(self.waypoint_keys_order))
            next_key = self.waypoint_keys_order[self.current_wp_index]
            next_wp = self.active_waypoints[next_key]
            self.get_logger().info(f"  → Sonraki hedef: {next_wp['name']}")

    def execute_tracking(self):
        """
        Hedef takip görevi - YOLO + GPS füzyon ile 4 saniye kilitlenme.
        
        Akış:
            1. Hedef tespit → 4s timer başla
            2. Visual servoing: bbox merkezi → yaw/pitch düzeltme
            3. Hedef GPS varsa → pozisyon takibi
            4. 4s tamamlanırsa → kilitlenme başarılı → PATROL'e dön
            5. Hedef kaybolursa → 2s bekle, kayıpsa → PATROL'e dön
        """
        TARGET_LOST_TIMEOUT = 2.0  # Hedef kayıp toleransı (s)

        # ═══ TRACKING BAŞLANGIÇ ═══
        if self.task_state == 0:
            self.get_logger().info(">> TRACKING: Hedef takibi başlatılıyor")
            self.tracking_start_time = None
            self.prev_yaw_error = 0.0
            self.task_state = 1

        elif self.task_state == 1:
            now = time.time()

            # ─── Hedef görünüyor mu? ───
            if self.target_detected and self.target_bbox:
                bbox = self.target_bbox
                self.target_lost_time = None  # Kayıp timer'ı sıfırla

                # Timer başlat (ilk tespit anında)
                if self.tracking_start_time is None:
                    self.tracking_start_time = now
                    self.get_logger().info("  🎯 KİLİTLENME BAŞLADI (4s)")

                # ─── Visual Servoing: BBox → Yaw/Pitch düzeltme ───
                # bbox x,y normalize [0,1] aralığında
                error_x = bbox['x'] - 0.5   # Pozitif = hedef sağda
                error_y = bbox['y'] - 0.5   # Pozitif = hedef aşağıda

                # PD kontrolcü → yaw düzeltme (derece)
                yaw_correction = (
                    TRACKING_YAW_KP * error_x +
                    TRACKING_YAW_KD * (error_x - self.prev_yaw_error)
                )
                self.prev_yaw_error = error_x

                # Pitch düzeltme (aşağı/yukarı)
                pitch_correction = -TRACKING_PITCH_KP * error_y

                # ─── Hedef GPS varsa → pozisyon takibi ───
                if self.target_gps:
                    tgt_x, tgt_y = self.global_to_local(
                        self.target_gps['lat'], self.target_gps['lon'])
                    tgt_alt = self.target_gps.get('alt', PATROL_ALT)
                    self.publish_position_with_velocity(
                        tgt_x, tgt_y, tgt_alt,
                        max_speed=TRACKING_SPEED,
                        target_x=tgt_x, target_y=tgt_y)
                else:
                    # GPS yok → mevcut yönde devam et, sadece yaw düzelt
                    self.publish_setpoint(
                        self.current_pos['x'],
                        self.current_pos['y'],
                        self.current_pos['z'])

                # ─── Kilitlenme kontrolü ───
                elapsed = now - self.tracking_start_time
                centered = (abs(error_x) < TRACKING_CENTER_TOL and
                            abs(error_y) < TRACKING_CENTER_TOL)

                if self.log_counter % 10 == 0:
                    status = "✓ MERKEZLİ" if centered else "⬚ DÜZELT"
                    self.get_logger().info(
                        f"   TRACKING | {elapsed:.1f}s / {TRACKING_LOCK_DURATION}s "
                        f"| errX:{error_x:+.2f} errY:{error_y:+.2f} "
                        f"| yaw:{yaw_correction:+.1f}° | {status}")
                self.log_counter += 1

                # 4 saniye tamamlandı!
                if elapsed >= TRACKING_LOCK_DURATION:
                    self.tracking_success_count += 1
                    self.get_logger().info(
                        f"  ✅ KİLİTLENME BAŞARILI! (#{self.tracking_success_count}) "
                        f"→ PATROL'e dönülüyor")
                    self.target_detected = False
                    self.target_bbox = None
                    self.tracking_start_time = None
                    self.finish_task()
                    return

            else:
                # ─── Hedef kayboldu ───
                if self.target_lost_time is None:
                    self.target_lost_time = now
                    self.get_logger().warn("  ⚠ HEDEF KAYBEDILDI - Bekleniyor...")

                # Mevcut pozisyonda kal
                self.publish_setpoint(
                    self.current_pos['x'],
                    self.current_pos['y'],
                    self.current_pos['z'])

                lost_duration = now - self.target_lost_time
                if lost_duration > TARGET_LOST_TIMEOUT:
                    self.get_logger().warn(
                        f"  ✖ HEDEF {TARGET_LOST_TIMEOUT:.0f}s KAYIP "
                        f"→ PATROL'e dönülüyor")
                    self.target_detected = False
                    self.target_bbox = None
                    self.tracking_start_time = None
                    self.finish_task()
                    return

                if self.log_counter % 20 == 0:
                    self.get_logger().info(
                        f"   TRACKING | Hedef kayıp {lost_duration:.1f}s "
                        f"/ {TARGET_LOST_TIMEOUT:.0f}s")
                self.log_counter += 1

    def execute_land(self):
        """AUTO.LAND modu ile home konumuna iniş."""
        if self.task_state == 0:
            self.get_logger().info(">> LAND: Home konumuna iniş yapılıyor")
            self.set_mode_command("AUTO.LAND")
            self.task_state = 1

        elif self.task_state == 1:
            if self.log_counter % 40 == 0:
                self.get_logger().info(
                    f"   LAND | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < 2.0:
                self.get_logger().info(">> İNİŞ TAMAMLANDI")
                self.finish_task()

    def finish_task(self):
        """Mevcut görevi tamamla."""
        self.get_logger().info(
            f"<< GÖREV TAMAMLANDI: {self.current_task['type']}")
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0

    # ═══════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ═══════════════════════════════════════════════════════════════

    def global_to_local(self, lat, lon):
        """GPS koordinatını local ENU koordinatına çevirir."""
        if not self.home_pos:
            return 0.0, 0.0
        R = 6371000.0
        home_lat = math.radians(self.home_pos['lat'])
        d_lat = math.radians(lat - self.home_pos['lat'])
        d_lon = math.radians(lon - self.home_pos['lon'])
        x = d_lon * R * math.cos(home_lat)  # East
        y = d_lat * R                        # North
        return x, y

    def gps_distance(self, lat1, lon1, lat2, lon2):
        """İki GPS noktası arasındaki mesafe (metre) - Haversine."""
        R = 6371000.0
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = (math.sin(d_lat / 2)**2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(d_lon / 2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def distance_to(self, x, y):
        """Hedef noktaya olan 2D mesafe (local ENU)."""
        dx = x - self.current_pos['x']
        dy = y - self.current_pos['y']
        return math.sqrt(dx**2 + dy**2)

    # ═══════════════════════════════════════════════════════════════
    # PUBLISH FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def publish_setpoint(self, x, y, z):
        """Local position setpoint yayınla."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.local_pos_pub.publish(msg)

    def publish_position_with_velocity(self, x, y, z,
                                        max_speed=None,
                                        target_x=None, target_y=None):
        """Konum + Hız kontrolü ile setpoint gönder."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        if max_speed is not None:
            tx = target_x if target_x is not None else x
            ty = target_y if target_y is not None else y
            dx = tx - self.current_pos['x']
            dy = ty - self.current_pos['y']
            dist = math.sqrt(dx**2 + dy**2)

            if dist > 0.1:
                vx = (dx / dist) * max_speed
                vy = (dy / dist) * max_speed
            else:
                vx, vy = 0.0, 0.0

            msg.velocity.x = float(vx)
            msg.velocity.y = float(vy)
            msg.velocity.z = 0.0
            msg.type_mask = 0b0000_1111_1100_0000
        else:
            msg.type_mask = 0b0000_1111_1111_1000

        self.setpoint_raw_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    # SERVİS FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def send_takeoff_command(self, relative_alt):
        """Takeoff komutu (AMSL irtifası kullanarak)."""
        req = CommandTOL.Request()
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.get_logger().info(
            f"   Takeoff AMSL: {req.altitude:.1f}m "
            f"(Home: {self.home_alt_amsl:.1f}m + Rel: {relative_alt:.1f}m)")
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

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z

    def global_position_callback(self, msg):
        """Global position callback - AMSL irtifasını kaydet."""
        self.current_gps = msg
        if self.home_alt_amsl == 0.0 and msg.altitude != 0:
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"   HOME AMSL: {self.home_alt_amsl:.1f}m")
        if not self.mavros_origin_set and self.home_pos:
            self.mavros_origin_set = True
            self.get_logger().info("   MAVROS origin set!")

    def home_position_callback(self, msg):
        if self.home_pos is None:
            self.home_pos = {
                'lat': msg.geo.latitude,
                'lon': msg.geo.longitude,
                'alt': msg.geo.altitude
            }
            self.get_logger().info(
                f"   Home position: {self.home_pos['lat']:.6f}, "
                f"{self.home_pos['lon']:.6f}")

    def yolo_detection_callback(self, msg):
        """
        YOLO tespit callback - vision_msgs/Detection2DArray formatı.
        En yüksek confidence'lı tespiti hedef olarak seçer.
        """
        if not msg.detections:
            self.target_detected = False
            self.target_bbox = None
            return

        # En yüksek score'lu tespiti bul
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.score > best_score:
                best_score = det.results[0].hypothesis.score
                best_det = det

        if best_det and best_score > 0.5:  # Min confidence eşiği
            bbox = best_det.bbox
            # Normalize edilmiş merkez koordinatları [0, 1]
            self.target_bbox = {
                'x': bbox.center.position.x / CAMERA_WIDTH,
                'y': bbox.center.position.y / CAMERA_HEIGHT,
                'w': bbox.size_x / CAMERA_WIDTH,
                'h': bbox.size_y / CAMERA_HEIGHT,
            }
            self.target_detected = True
        else:
            self.target_detected = False
            self.target_bbox = None

    def target_gps_callback(self, msg):
        """Rakip İHA GPS callback."""
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.target_gps = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = HSSWaypointMission()

    # ═══════════════════════════════════════════════════════════════
    # TEST SENARYOSU
    # ═══════════════════════════════════════════════════════════════
    # Aşağıdaki timer'lar operatör müdahalesini simüle eder.
    # İstediğin satırların başındaki # işaretini kaldırarak aktif et.
    # Timer'lar bir kez çalışır (one-shot), sonra kendini iptal eder.
    #
    # SENARYO:
    #   t=0s    → Node başlar, TAKEOFF + PATROL otomatik başlar
    #   t=30s   → HSS bölgesi aktive edilir (WP2 yakını)
    #   t=40s   → HSS efektif olur (10s gecikme). Uçak WP2'yi atlar
    #   t=50s   → Yeni waypoint eklenir (WP5)
    #   t=100s  → HSS kapanır (60s aktiflik). Uçak WP2'ye gidebilir
    #   t=150s  → LAND komutu → Uçak home'a iner
    # ═══════════════════════════════════════════════════════════════

    # ─── Yardımcı: Tek seferlik timer oluşturucu ───
    def one_shot(delay_sec, callback):
        """Belirtilen süre sonra bir kez çalışıp kapanan timer."""
        timer_holder = {}
        def wrapper():
            callback()
            timer_holder['timer'].cancel()  # Kendini iptal et
        timer_holder['timer'] = node.create_timer(delay_sec, wrapper)

    # ┌─────────────────────────────────────────────────────────┐
    # │ SENARYO 1: HSS TEST (yorum işaretlerini kaldırarak aç) │
    # └─────────────────────────────────────────────────────────┘

    # 30 saniye sonra → WP2 yakınında HSS bölgesi aktive et
    one_shot(30.0, lambda: node.activate_hss(
        zone_id=1,
        lat=40.229318,       # WP2'nin koordinatları
        lon=29.009345,
        radius=200.0         # 200m yarıçap
    ))

    # 50 saniye sonra → Yeni bir waypoint ekle
    one_shot(50.0, lambda: node.add_waypoint(
        key=5,
        lat=40.231500,
        lon=29.004000,
        alt=100.0,
        name="WP5_Yeni"
    ))

    # 150 saniye sonra → LAND komutu (home'a iniş)
    one_shot(150.0, lambda: node.command_land())

    # ┌─────────────────────────────────────────────────────────┐
    # │ SENARYO 2: SADECE PATROL (hiçbir müdahale yok)         │
    # │ → Yorum satırlarını olduğu gibi bırak, uçak sadece     │
    # │   4 köşeyi döngüsel olarak dolaşır                     │
    # └─────────────────────────────────────────────────────────┘

    # ┌─────────────────────────────────────────────────────────┐
    # │ SENARYO 3: HIZLI TEST (kısa süreli)                    │
    # └─────────────────────────────────────────────────────────┘

    # # 20s sonra HSS aç
    # one_shot(20.0, lambda: node.activate_hss(1, 40.229318, 29.009345, 150.0))
    # # 60s sonra waypoint sil
    # one_shot(60.0, lambda: node.remove_waypoint(1))
    # # 90s sonra iniş
    # one_shot(90.0, lambda: node.command_land())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("HSS Waypoint Mission durduruldu.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
