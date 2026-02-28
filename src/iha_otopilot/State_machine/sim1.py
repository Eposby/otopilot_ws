#!/usr/bin/env python3
"""
FW_THR_IDLE (15%): Uçuş sırasında throttle stick'i en aşağıda olsa bile motor tamamen kapanmaz. Ani güç gerektiğinde gecikme olmaz


# Maksimum airspeed - bu değeri değiştir
param set FW_AIRSPD_MAX 25.0      # Varsayılan çok yüksek olabilir

# Minimum airspeed (stall koruması)
param set FW_AIRSPD_MIN 10.0

# Trim (cruise) airspeed
param set FW_AIRSPD_TRIM 15.0

# Takeoff sırasındaki throttle
param set FW_THR_IDLE 0.15        # Rölanti
param set FW_THR_MAX 1.0          # Maksimum
param set FW_THR_SLEW_MAX 0.0     # Throttle geçiş hızı (0=sınırsız)

# Tırmanış hızı limiti
param set FW_T_CLMB_MAX 10.0      # m/s

# Takeoff pitch
param set FW_TKO_PITCH_MIN 10.0   # Minimum takeoff pitch (derece)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, AttitudeTarget, HomePosition, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import math


# ============================================================================
# WAYPOINT VE QR KONFİGÜRASYONU
# ============================================================================

# QR Hedef Koordinatları (Kamikaze yapılacak nokta)
QR_LAT = 40.230712658763466
QR_LON = 29.006760026391138
QR_PROXIMITY = 300.0  # Bu mesafeden yakın olunca kamikaze tetiklenir (metre)

# Devriye Waypoint Listesi (GPS koordinatları)
# Format: {'name': 'WP1', 'lat': xx.xxx, 'lon': yy.yyy, 'alt': zz.z}
WAYPOINTS = [
    {'name': 'WP1', 'lat': 40.229500, 'lon': 29.005500, 'alt': 100.0},
    {'name': 'WP2', 'lat': 40.230000, 'lon': 29.008000, 'alt': 100.0},
    {'name': 'WP3', 'lat': 40.231500, 'lon': 29.007500, 'alt': 100.0},
    {'name': 'WP4', 'lat': 40.231000, 'lon': 29.005000, 'alt': 100.0},
]

# Genel Parametreler
TAKEOFF_ALT = 100.0
PATROL_ALT = 100.0
WAYPOINT_RADIUS = 50.0  # Waypoint'e ulaşıldı sayılma mesafesi (metre)

# Hız Parametreleri
PATROL_SPEED = 18.0           # Patrol hızı (m/s)
APPROACH_SPEED = 22.0         # Kamikaze yaklaşma hızı (m/s)

# Kamikaze Öncesi Yavaşlama
SLOWDOWN_THRUST = 0.5       
SLOWDOWN_DURATION = 80       # 6 saniye (120 × 50ms)

# Dalış Parametreleri (ENU frame - pozitif = burun aşağı)
PULL_UP_ALT = 40.0            # Pull-up başlangıç irtifası
DIVE_PITCH_1 = 45.0           
DIVE_PITCH_2 = 45.0           
DIVE_PITCH_3 = 45.0           
DIVE_THRUST = 0.5           

# Pull-up Parametreleri
SAFE_ALT = 90.0
PULLUP_PITCH_1 = -30.0        
PULLUP_PITCH_2 = -20.0        
PULLUP_PITCH_3 = -10.0        
PULLUP_PITCH_4 = 0.0          
PULLUP_ROLL = 45.0            
PULLUP_THRUST = 1.0           


# ============================================================================
# SIM1 NODE SINIFI
# ============================================================================

class Sim1(Node):

    def __init__(self):
        super().__init__('sim1_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_position_callback, qos_profile)
        self.home_sub = self.create_subscription(
            HomePosition, '/mavros/home_position/home', self.home_position_callback, qos_profile)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.setpoint_raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # State Variables
        self.current_state = None
        self.current_gps = None
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0  # Home noktasının AMSL irtifası

        # Mission Variables
        self.mission_queue = []
        self.current_task = None
        self.task_state = 0
        
        # Waypoint Patrol Variables
        self.current_wp_index = 0           # Şu anki waypoint indeksi
        self.patrol_waypoints_local = []    # Local koordinatlar
        self.kamikaze_completed = False     # Kamikaze yapıldı mı?
        self.qr_x = 0.0                     # QR local X
        self.qr_y = 0.0                     # QR local Y

        self.log_counter = 0
        self.timeout_counter = 0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SIM1 Waypoint Patrol + Kamikaze Node başlatıldı")


    # ========================================================================
    # CALLBACK FONKSİYONLARI
    # ========================================================================

    def state_callback(self, msg):
        self.current_state = msg

    def global_position_callback(self, msg):
        """Global position callback - AMSL irtifasını kaydet."""
        self.current_gps = msg
        # İlk geçerli GPS'te home AMSL'i kaydet
        if self.home_alt_amsl == 0.0 and msg.altitude != 0:
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"HOME AMSL: {self.home_alt_amsl:.1f}m")
        if not self.mavros_origin_set and self.home_pos:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS origin set!")

    def local_position_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z

    def home_position_callback(self, msg):
        if self.home_pos is None:
            self.home_pos = {
                'lat': msg.geo.latitude,
                'lon': msg.geo.longitude,
                'alt': msg.geo.altitude
            }
            self.get_logger().info(f"Home position set: {self.home_pos['lat']:.6f}, {self.home_pos['lon']:.6f}")


    # ========================================================================
    # YARDIMCI FONKSİYONLAR
    # ========================================================================

    def global_to_local(self, lat, lon):
        """GPS koordinatını local ENU koordinatına çevirir."""
        if not self.home_pos:
            return 0.0, 0.0

        R = 6371000  # Earth radius in meters
        home_lat = math.radians(self.home_pos['lat'])
        home_lon = math.radians(self.home_pos['lon'])
        target_lat = math.radians(lat)
        target_lon = math.radians(lon)

        x = R * (target_lon - home_lon) * math.cos(home_lat)  # East
        y = R * (target_lat - home_lat)  # North
        return x, y

    def distance_to(self, x, y):
        """Hedef noktaya olan yatay mesafeyi hesaplar."""
        dx = x - self.current_pos['x']
        dy = y - self.current_pos['y']
        return math.sqrt(dx**2 + dy**2)

    def yaw_to(self, x, y):
        """Hedef noktaya yaw açısını hesaplar (derece)."""
        dx = x - self.current_pos['x']
        dy = y - self.current_pos['y']
        return math.degrees(math.atan2(dy, dx))

    def finish_task(self):
        """Mevcut görevi tamamla."""
        self.get_logger().info(f"<< GÖREV TAMAMLANDI: {self.current_task['type']}")
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0


    # ========================================================================
    # SERVİS FONKSİYONLARI
    # ========================================================================

    def set_mode_command(self, mode):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode servisi bekleniyor...')
        
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        return future

    def send_arm_command(self, arm_status):
        """Arm/Disarm komutu gönder."""
        req = CommandBool.Request()
        req.value = arm_status
        self.arming_client.call_async(req)

    def send_takeoff_command(self, relative_alt):
        """Takeoff komutu gönder (AMSL irtifası kullanarak)."""
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff servisi bekleniyor...')
        
        req = CommandTOL.Request()
        # AMSL irtifası = Home AMSL + göreceli irtifa
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.get_logger().info(f"Takeoff AMSL: {req.altitude:.1f}m (Home: {self.home_alt_amsl:.1f}m + Rel: {relative_alt:.1f}m)")
        future = self.takeoff_client.call_async(req)
        return future


    # ========================================================================
    # MİSYON SETUP
    # ========================================================================

    def setup_mission(self):
        """Misyon kurulumu - waypointleri local koordinatlara çevir."""
        if not self.home_pos:
            return

        # QR koordinatlarını local'a çevir
        self.qr_x, self.qr_y = self.global_to_local(QR_LAT, QR_LON)
        self.get_logger().info(f"QR Hedef: ({self.qr_x:.0f}, {self.qr_y:.0f})")

        # Waypointleri local koordinatlara çevir
        self.patrol_waypoints_local = []
        for wp in WAYPOINTS:
            x, y = self.global_to_local(wp['lat'], wp['lon'])
            self.patrol_waypoints_local.append({
                'name': wp['name'],
                'x': x,
                'y': y,
                'alt': wp['alt']
            })
            self.get_logger().info(f"  {wp['name']}: ({x:.0f}, {y:.0f}, {wp['alt']:.0f}m)")

        # Misyon kuyruğu - TAKEOFF sonrası PATROL'e geç
        self.mission_queue.append({'type': 'TAKEOFF', 'alt': TAKEOFF_ALT})
        self.mission_queue.append({'type': 'PATROL'})


    # ========================================================================
    # ANA KONTROL DÖNGÜSÜ
    # ========================================================================

    def control_loop(self):
        """50ms'de bir çalışan ana kontrol döngüsü."""
        if not self.home_pos or not self.mavros_origin_set:
            return

        if len(self.mission_queue) == 0 and self.current_task is None and self.task_state == 0:
            self.setup_mission()

        if self.current_task is None:
            if len(self.mission_queue) > 0:
                self.current_task = self.mission_queue.pop(0)
                self.task_state = 0
                self.log_counter = 0
                self.timeout_counter = 0
                self.get_logger().info(f"\n>> YENİ GÖREV: {self.current_task['type']}")
            else:
                return

        task_type = self.current_task['type']

        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'PATROL':
            self.execute_patrol()
        elif task_type == 'SLOWDOWN':
            self.execute_slowdown()
        elif task_type == 'KAMIKAZE':
            self.execute_kamikaze()
        elif task_type == 'RETURN_TO_PATROL':
            self.execute_return_to_patrol()


    # ========================================================================
    # GÖREV FONKSİYONLARI
    # ========================================================================

    def execute_takeoff(self):
        """Kalkış görevi - AUTO.TAKEOFF modu ve ARM."""
        target_alt = self.current_task['alt']

        if self.task_state == 0:
            self.get_logger().info(f">> TAKEOFF: Hedef irtifa {target_alt}m")
            self.send_takeoff_command(target_alt)
            self.task_state = 1

        elif self.task_state == 1:
            # AUTO.TAKEOFF modunu bekle
            if self.current_state and self.current_state.mode == "AUTO.TAKEOFF":
                self.get_logger().info(">> AUTO.TAKEOFF aktif - ARM ediliyor")
                self.send_arm_command(True)
                self.task_state = 2

        elif self.task_state == 2:
            # İrtifa takibi
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   TAKEOFF | İrtifa: {self.current_pos['z']:.0f}m / {target_alt}m")
            self.log_counter += 1

            if self.current_pos['z'] > target_alt - 5:
                self.get_logger().info(">> TAKEOFF TAMAMLANDI")
                self.finish_task()


    def execute_patrol(self):
        """Waypoint devriyesi - QR yakınlığını da kontrol eder."""
        
        if len(self.patrol_waypoints_local) == 0:
            return

        # State 0: OFFBOARD moduna geç
        if self.task_state == 0:
            self.get_logger().info(">> PATROL: OFFBOARD moduna geçiliyor")
            # Önce mevcut konumu setpoint olarak gönder (OFFBOARD için gerekli)
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z'])
            self.set_mode_command("OFFBOARD")
            self.task_state = 1
            return

        # State 1: Waypoint takibi
        # Mevcut waypoint
        wp = self.patrol_waypoints_local[self.current_wp_index]
        wp_name = wp['name']
        wp_x, wp_y, wp_alt = wp['x'], wp['y'], wp['alt']
        
        # Waypoint'e git
        dist_to_wp = self.distance_to(wp_x, wp_y)
        self.publish_position_with_velocity(wp_x, wp_y, wp_alt, max_speed=PATROL_SPEED, target_x=wp_x, target_y=wp_y)

        if self.log_counter % 20 == 0:
            # QR'a olan mesafeyi de logla
            dist_to_qr = self.distance_to(self.qr_x, self.qr_y)
            self.get_logger().info(f"   PATROL → {wp_name} | WP mesafe: {dist_to_wp:.0f}m | QR mesafe: {dist_to_qr:.0f}m | İrtifa: {self.current_pos['z']:.0f}m")
        self.log_counter += 1

        # QR YAKINLIK KONTROLÜ - Kamikaze tetikle
        dist_to_qr = self.distance_to(self.qr_x, self.qr_y)
        if dist_to_qr < QR_PROXIMITY and not self.kamikaze_completed:
            self.get_logger().info(f">> QR'A {dist_to_qr:.0f}m YAKINLIKTA! KAMİKAZE TETİKLENİYOR")
            self.kamikaze_completed = True  # Bir kez yapılsın
            # Mevcut görevi bitir, kamikaze sırasına ekle
            self.mission_queue.insert(0, {'type': 'SLOWDOWN'})
            self.mission_queue.insert(1, {'type': 'KAMIKAZE'})
            self.mission_queue.insert(2, {'type': 'RETURN_TO_PATROL'})
            self.finish_task()
            return

        # Waypoint'e ulaşıldı mı?
        if dist_to_wp < WAYPOINT_RADIUS:
            self.get_logger().info(f">> {wp_name} ULAŞILDI!")
            # Bir sonraki waypoint'e geç (döngü)
            self.current_wp_index = (self.current_wp_index + 1) % len(self.patrol_waypoints_local)
            next_wp = self.patrol_waypoints_local[self.current_wp_index]
            self.get_logger().info(f">> Sonraki hedef: {next_wp['name']}")


    def execute_slowdown(self):
        """Kamikaze öncesi yavaşlama fazı."""
        yaw = self.yaw_to(self.qr_x, self.qr_y)
        
        if self.task_state == 0:
            self.get_logger().info(f">> SLOWDOWN: Minimum hıza düşürülüyor")
            self.timeout_counter = 0
            self.task_state = 1

        elif self.task_state == 1:
            self.publish_attitude(roll=0.0, pitch=0.0, yaw=yaw, thrust=SLOWDOWN_THRUST)
            self.timeout_counter += 1
            
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   SLOWDOWN | Süre: {self.timeout_counter}/{SLOWDOWN_DURATION} | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            
            if self.timeout_counter >= SLOWDOWN_DURATION:
                self.get_logger().info(">> YAVAŞLAMA TAMAMLANDI - KAMİKAZE BAŞLIYOR")
                self.finish_task()


    def execute_kamikaze(self):
        """Kamikaze dalış manevrası - qr2.py ile aynı."""
        yaw = self.yaw_to(self.qr_x, self.qr_y)

        if self.task_state == 0:  # Phase 1: Yumuşak dalış
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_1, yaw=yaw, thrust=DIVE_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 1 | Pitch: +{DIVE_PITCH_1}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] < 100:
                self.task_state = 1

        elif self.task_state == 1:  # Phase 2
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_2, yaw=yaw, thrust=DIVE_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 2 | Pitch: +{DIVE_PITCH_2}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] < 75:
                self.task_state = 2

        elif self.task_state == 2:  # Phase 3
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_3, yaw=yaw, thrust=DIVE_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 3 | Pitch: +{DIVE_PITCH_3}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] < (PULL_UP_ALT + 5):
                self.get_logger().info(f">> {PULL_UP_ALT}m - PULL-UP BAŞLIYOR")
                self.task_state = 3

        elif self.task_state == 3:  # Pull-up Phase 1
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_1, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 1 | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 60.0:
                self.task_state = 4

        elif self.task_state == 4:  # Pull-up Phase 2
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_2, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 2 | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 70.0:
                self.task_state = 5

        elif self.task_state == 5:  # Pull-up Phase 3
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_3, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 3 | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 80.0:
                self.task_state = 6

        elif self.task_state == 6:  # Pull-up Phase 4 - düzleşme
            self.publish_attitude(roll=0.0, pitch=PULLUP_PITCH_4, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 4 | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > SAFE_ALT:
                self.get_logger().info(">> KAMİKAZE TAMAMLANDI - PATROL'E DÖNÜLÜYOR")
                self.finish_task()


    def execute_return_to_patrol(self):
        """Kamikaze sonrası patrol'e geri dön."""
        if self.task_state == 0:
            self.get_logger().info(">> RETURN_TO_PATROL: Devriye moduna dönülüyor")
            # Patrol görevini tekrar kuyruğa ekle
            self.mission_queue.append({'type': 'PATROL'})
            self.finish_task()


    # ========================================================================
    # PUBLISH FONKSİYONLARI
    # ========================================================================

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Euler açılarından quaternion'a dönüşüm."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

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

    def publish_position_with_velocity(self, x, y, z, max_speed=None, target_x=None, target_y=None):
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

    def publish_attitude(self, roll, pitch, yaw, thrust):
        """Attitude setpoint yayınla."""
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # type_mask = 7: IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE
        # Bu sayede orientation ve thrust kullanılır
        msg.type_mask = 7

        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.thrust = float(thrust)

        self.attitude_pub.publish(msg)


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Sim1()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SIM1 Node durduruldu.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
