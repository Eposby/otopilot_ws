#!/usr/bin/env python3
"""
Enemy Plane Mission - Hayali Düşman İHA Otonom Uçuş Misyonu
============================================================

Bu node, simülasyondaki hayali düşman uçakları otonom olarak
operasyon alanında devriye gezdirmek için kullanılır.

Her uçak için ayrı bir instance çalıştırılır, namespace parametresi
ile hangi MAVROS instance'ına bağlanacağı belirlenir.

Kullanım:
    # Düşman uçak 0 (namespace: uav0, varsayılan irtifa 80m):
    ros2 run iha_otopilot enemy_plane --ros-args \
        -p namespace:="uav0" -p vehicle_id:=0

    # Düşman uçak 1 (namespace: uav1, varsayılan irtifa 120m):
    ros2 run iha_otopilot enemy_plane --ros-args \
        -p namespace:="uav1" -p vehicle_id:=1

Özellikler:
    - Namespace'e göre MAVROS topic'lerini otomatik ayarlar
    - TAKEOFF → PATROL (waypoint döngüsü) state machine
    - Farklı irtifa/hız profilleri (çarpışma önleme)
    - Operasyon alanı köşeleri arası devriye
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, HomePosition, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import math


# ═══════════════════════════════════════════════════════════════
# OPERASYON ALANI WAYPOINT'LERİ
# ═══════════════════════════════════════════════════════════════
# HSS yarışma alanı köşeleri
OPERATION_WAYPOINTS = [
    {'name': 'WP1_Kose1', 'lat': 40.230761, 'lon': 29.001866},
    {'name': 'WP2_Kose2', 'lat': 40.229318, 'lon': 29.009345},
    {'name': 'WP3_Kose3', 'lat': 40.233739, 'lon': 29.009292},
    {'name': 'WP4_Kose4', 'lat': 40.233848, 'lon': 28.998916},
]

# ═══════════════════════════════════════════════════════════════
# UÇAK PROFİLLERİ (vehicle_id'ye göre farklılaşır)
# ═══════════════════════════════════════════════════════════════
# Her uçak farklı irtifa ve hızda uçar → çarpışma önleme
VEHICLE_PROFILES = {
    0: {'alt': 80.0,  'speed': 15.0, 'start_wp': 0},
    1: {'alt': 120.0, 'speed': 18.0, 'start_wp': 2},
    2: {'alt': 100.0, 'speed': 16.0, 'start_wp': 1},
    3: {'alt': 90.0,  'speed': 17.0, 'start_wp': 3},
    4: {'alt': 110.0, 'speed': 14.0, 'start_wp': 0},
}

# Genel Parametreler
WAYPOINT_RADIUS = 50.0       # Waypoint'e ulaşıldı sayılma mesafesi (m)


class EnemyPlaneMission(Node):

    def __init__(self):
        super().__init__('enemy_plane_mission_node')

        # ═══════════════════════════════════════════════════════
        # PARAMETRELER
        # ═══════════════════════════════════════════════════════
        self.declare_parameter('namespace', 'uav0')
        self.declare_parameter('vehicle_id', 0)

        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        # Profil seç (bilinmeyen id → varsayılan profil)
        profile = VEHICLE_PROFILES.get(self.vehicle_id, {
            'alt': 100.0, 'speed': 16.0, 'start_wp': 0
        })
        self.patrol_alt = profile['alt']
        self.patrol_speed = profile['speed']
        self.start_wp_index = profile['start_wp']

        self.get_logger().info(f"═══ DÜŞMAN UÇAK MİSYONU ═══")
        self.get_logger().info(f"  Namespace: /{self.ns}")
        self.get_logger().info(f"  Vehicle ID: {self.vehicle_id}")
        self.get_logger().info(f"  İrtifa: {self.patrol_alt}m")
        self.get_logger().info(f"  Hız: {self.patrol_speed} m/s")
        self.get_logger().info(f"  Başlangıç WP: {self.start_wp_index}")

        # ═══════════════════════════════════════════════════════
        # QOS PROFİLİ
        # ═══════════════════════════════════════════════════════
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ═══════════════════════════════════════════════════════
        # TOPIC İSİMLERİ (namespace'e göre)
        # ═══════════════════════════════════════════════════════
        prefix = f"/{self.ns}/mavros"

        # ─── SUBSCRIBERS ───
        self.state_sub = self.create_subscription(
            State, f'{prefix}/state',
            self.state_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, f'{prefix}/global_position/global',
            self.global_position_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, f'{prefix}/local_position/pose',
            self.local_position_callback, qos_profile)
        self.home_sub = self.create_subscription(
            HomePosition, f'{prefix}/home_position/home',
            self.home_position_callback, qos_profile)

        # ─── PUBLISHERS ───
        self.local_pos_pub = self.create_publisher(
            PoseStamped, f'{prefix}/setpoint_position/local', 10)
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget, f'{prefix}/setpoint_raw/local', 10)

        # ─── SERVICE CLIENTS ───
        self.arming_client = self.create_client(
            CommandBool, f'{prefix}/cmd/arming')
        self.takeoff_client = self.create_client(
            CommandTOL, f'{prefix}/cmd/takeoff')
        self.set_mode_client = self.create_client(
            SetMode, f'{prefix}/set_mode')

        # ═══════════════════════════════════════════════════════
        # STATE DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════
        self.current_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gps = None
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0

        # ═══════════════════════════════════════════════════════
        # MİSYON DEĞİŞKENLERİ
        # ═══════════════════════════════════════════════════════
        self.mission_queue = []
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0

        # Waypoint takibi
        self.current_wp_index = self.start_wp_index
        self.patrol_waypoints_local = []
        self.mission_setup_done = False

        # ═══════════════════════════════════════════════════════
        # KONTROL DÖNGÜSÜ (50ms = 20 Hz)
        # ═══════════════════════════════════════════════════════
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"  ✅ Node başlatıldı, MAVROS bağlantısı bekleniyor...")

    # ═══════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════

    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z

    def global_position_callback(self, msg):
        self.current_gps = msg
        if self.home_alt_amsl == 0.0 and msg.altitude != 0:
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"  HOME AMSL: {self.home_alt_amsl:.1f}m")
        if not self.mavros_origin_set and self.home_pos:
            self.mavros_origin_set = True
            self.get_logger().info(f"  MAVROS origin set! [{self.ns}]")

    def home_position_callback(self, msg):
        if self.home_pos is None:
            self.home_pos = {
                'lat': msg.geo.latitude,
                'lon': msg.geo.longitude,
                'alt': msg.geo.altitude
            }
            self.get_logger().info(
                f"  Home: {self.home_pos['lat']:.6f}, "
                f"{self.home_pos['lon']:.6f}")

    # ═══════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ═══════════════════════════════════════════════════════════

    def global_to_local(self, lat, lon):
        """GPS → Local ENU koordinat dönüşümü."""
        if not self.home_pos:
            return 0.0, 0.0
        R = 6371000.0
        home_lat = math.radians(self.home_pos['lat'])
        d_lat = math.radians(lat - self.home_pos['lat'])
        d_lon = math.radians(lon - self.home_pos['lon'])
        x = d_lon * R * math.cos(home_lat)  # East
        y = d_lat * R                        # North
        return x, y

    def distance_to(self, x, y):
        """2D mesafe hesabı."""
        dx = x - self.current_pos['x']
        dy = y - self.current_pos['y']
        return math.sqrt(dx**2 + dy**2)

    def finish_task(self):
        """Mevcut görevi tamamla."""
        self.get_logger().info(
            f"  << GÖREV TAMAMLANDI: {self.current_task['type']} [{self.ns}]")
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0

    # ═══════════════════════════════════════════════════════════
    # SERVİS FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════

    def send_takeoff_command(self, relative_alt):
        """Takeoff komutu (AMSL hesabıyla)."""
        req = CommandTOL.Request()
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.get_logger().info(
            f"  Takeoff AMSL: {req.altitude:.1f}m "
            f"(Home: {self.home_alt_amsl:.1f}m + Rel: {relative_alt:.1f}m)")
        self.takeoff_client.call_async(req)

    def send_arm_command(self, arm_status):
        """ARM/DISARM komutu."""
        req = CommandBool.Request()
        req.value = arm_status
        self.arming_client.call_async(req)

    def set_mode_command(self, mode):
        """Uçuş modu değiştir."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    # ═══════════════════════════════════════════════════════════
    # PUBLISH FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════

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

    # ═══════════════════════════════════════════════════════════
    # MİSYON SETUP
    # ═══════════════════════════════════════════════════════════

    def setup_mission(self):
        """Misyon kurulumu - waypoint'leri local koordinatlara çevir."""
        if not self.home_pos:
            return

        self.get_logger().info(f"═══ MİSYON KURULUMU [{self.ns}] ═══")

        # Waypoint'leri local ENU koordinatlarına çevir
        self.patrol_waypoints_local = []
        for wp in OPERATION_WAYPOINTS:
            x, y = self.global_to_local(wp['lat'], wp['lon'])
            self.patrol_waypoints_local.append({
                'name': wp['name'],
                'x': x,
                'y': y,
                'alt': self.patrol_alt
            })
            self.get_logger().info(
                f"  {wp['name']}: ({x:.0f}, {y:.0f}, {self.patrol_alt:.0f}m)")

        # İndeks sınır kontrolü
        if self.current_wp_index >= len(self.patrol_waypoints_local):
            self.current_wp_index = 0

        # Misyon kuyruğu
        self.mission_queue.append({'type': 'TAKEOFF', 'alt': self.patrol_alt})
        self.mission_queue.append({'type': 'PATROL'})
        self.mission_setup_done = True

    # ═══════════════════════════════════════════════════════════
    # ANA KONTROL DÖNGÜSÜ
    # ═══════════════════════════════════════════════════════════

    def control_loop(self):
        """50ms kontrol döngüsü."""
        if not self.home_pos or not self.mavros_origin_set:
            return

        # İlk çalışmada misyonu kur
        if not self.mission_setup_done:
            self.setup_mission()

        # Yeni göreve geç
        if self.current_task is None:
            if len(self.mission_queue) > 0:
                self.current_task = self.mission_queue.pop(0)
                self.task_state = 0
                self.log_counter = 0
                self.get_logger().info(
                    f"\n>> YENİ GÖREV: {self.current_task['type']} [{self.ns}]")
            else:
                return

        # Görev yönlendirme
        task_type = self.current_task['type']

        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'PATROL':
            self.execute_patrol()

    # ═══════════════════════════════════════════════════════════
    # GÖREV FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════

    def execute_takeoff(self):
        """Kalkış → AUTO.TAKEOFF → ARM → irtifa bekle."""
        target_alt = self.current_task['alt']

        if self.task_state == 0:
            self.get_logger().info(
                f">> TAKEOFF [{self.ns}]: Hedef irtifa {target_alt}m")
            self.send_takeoff_command(target_alt)
            self.task_state = 1

        elif self.task_state == 1:
            if self.current_state.mode == "AUTO.TAKEOFF":
                self.get_logger().info(
                    f">> AUTO.TAKEOFF aktif [{self.ns}] - ARM ediliyor")
                self.send_arm_command(True)
                self.task_state = 2

        elif self.task_state == 2:
            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"   TAKEOFF [{self.ns}] | İrtifa: "
                    f"{self.current_pos['z']:.0f}m / {target_alt}m")
            self.log_counter += 1

            if self.current_pos['z'] > (target_alt - 5.0):
                self.get_logger().info(
                    f">> TAKEOFF TAMAMLANDI [{self.ns}]")
                self.finish_task()

    def execute_patrol(self):
        """
        Waypoint devriye - operasyon alanı köşeleri arası döngüsel uçuş.
        Waypoint'e ulaşınca otomatik olarak sonrakine geçer.
        Sonsuz döngü olarak çalışır (sürekli devriye).
        """
        if len(self.patrol_waypoints_local) == 0:
            return

        # ─── OFFBOARD moduna geç (ilk çağrı) ───
        if self.task_state == 0:
            self.get_logger().info(
                f">> PATROL [{self.ns}]: OFFBOARD moduna geçiliyor "
                f"(Alt: {self.patrol_alt}m, Hız: {self.patrol_speed} m/s)")
            self.publish_setpoint(
                self.current_pos['x'],
                self.current_pos['y'],
                self.current_pos['z'])
            self.set_mode_command("OFFBOARD")
            self.task_state = 1
            return

        # ─── Waypoint takibi ───
        wp = self.patrol_waypoints_local[self.current_wp_index]
        wp_x, wp_y, wp_alt = wp['x'], wp['y'], wp['alt']

        # Hedefe git (hız limitli)
        dist_to_wp = self.distance_to(wp_x, wp_y)
        self.publish_position_with_velocity(
            wp_x, wp_y, wp_alt,
            max_speed=self.patrol_speed,
            target_x=wp_x, target_y=wp_y)

        # Loglama
        if self.log_counter % 40 == 0:  # Her 2 saniyede bir log
            self.get_logger().info(
                f"   PATROL [{self.ns}] → {wp['name']} | "
                f"Mesafe: {dist_to_wp:.0f}m | "
                f"Alt: {self.current_pos['z']:.0f}m | "
                f"WP: {self.current_wp_index+1}/"
                f"{len(self.patrol_waypoints_local)}")
        self.log_counter += 1

        # Waypoint'e ulaşıldı mı?
        if dist_to_wp < WAYPOINT_RADIUS:
            self.get_logger().info(
                f"  ✓ {wp['name']} ULAŞILDI! [{self.ns}]")
            # Sonraki waypoint'e geç (döngüsel)
            self.current_wp_index = (
                (self.current_wp_index + 1)
                % len(self.patrol_waypoints_local))
            next_wp = self.patrol_waypoints_local[self.current_wp_index]
            self.get_logger().info(
                f"  → Sonraki: {next_wp['name']} [{self.ns}]")


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = EnemyPlaneMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"Düşman uçak misyonu durduruldu [{node.ns}]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
