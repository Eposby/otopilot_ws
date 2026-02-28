#!/usr/bin/env python3
"""
QR2 Kamikaze Mission Node

GÖREV SIRASI:
1. TAKEOFF - Belirtilen irtifaya kalkış
2. GOTO_WP1 - QR'ın 300m gerisindeki waypointе git
3. WP2 - QR'a hızla yaklaş, 200m kala sonlandır
4. KAMIKAZE - Kademeli dalış ve pull-up
5. RTL - Eve dön

source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, AttitudeTarget, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import math



QR_LAT = 40.230712658763466
QR_LON = 29.006760026391138
TAKEOFF_ALT = 100.0
WP1_ALT = 150.0       
WP1_DISTANCE = 320.0      
WP2_DISTANCE = 200.0     
PULL_UP_ALT = 60.0       
SAFE_ALT = 100.0          


DIVE_PITCH_1 = 40.0      
DIVE_PITCH_2 = 40.0      
DIVE_PITCH_3 = 50.0      
DIVE_THRUST = 0.8       
PULLUP_PITCH_1 = -30.0    # 60-70m arası
PULLUP_PITCH_2 = -20.0    # 70-80m arası
PULLUP_PITCH_3 = -10.0    # 80-90m arası
PULLUP_PITCH_4 = 0.0      # 90m+ düz uçuş
PULLUP_ROLL = 45.0        # Kaçış roll açısı
PULLUP_THRUST = 1.0      


class QR2(Node):

    def __init__(self):
        super().__init__('qr2_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_position_callback, qos_profile)
        self.home_sub = self.create_subscription(
            HomePosition, '/mavros/home_position/home', self.home_position_callback, qos_profile)


        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)


        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')


        self.current_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0


        self.mission_queue = []
        self.current_task = None
        self.task_state = 0

        self.qr_x = 0.0
        self.qr_y = 0.0
        self.wp1_x = 0.0
        self.wp1_y = 0.0

        self.log_counter = 0
        self.timeout_counter = 0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("QR2")
        self.get_logger().info("dümenden info")



    def setup_mission(self):
 
        if not self.home_pos:
            return

        self.qr_x, self.qr_y = self.global_to_local(QR_LAT, QR_LON)
        
        dx = self.qr_x - self.current_pos['x']
        dy = self.qr_y - self.current_pos['y']
        dist = math.sqrt(dx**2 + dy**2)

        if dist > 0:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 0.0, 1.0

        self.wp1_x = self.qr_x - ux * WP1_DISTANCE
        self.wp1_y = self.qr_y - uy * WP1_DISTANCE

        self.get_logger().info(f"QR Konumu: ({self.qr_x:.0f}, {self.qr_y:.0f})")
        self.get_logger().info(f"WP1 Konumu: ({self.wp1_x:.0f}, {self.wp1_y:.0f})")


        self.mission_queue.append({'type': 'TAKEOFF', 'alt': TAKEOFF_ALT})
        self.mission_queue.append({'type': 'GOTO_WP1'})
        self.mission_queue.append({'type': 'WP2'})
        self.mission_queue.append({'type': 'KAMIKAZE'})
        self.mission_queue.append({'type': 'STABILIZE'})  # RTL öncesi stabilizasyon
        self.mission_queue.append({'type': 'RTL'})
        self.mission_queue.append({'type': 'LAND'})



    def control_loop(self):
        """
        50ms'de bir çalışan ana kontrol döngüsü
        """
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
                self.get_logger().info(f"\n>> YENİ GÖREV BAŞLADI: {self.current_task['type']}")
            else:
                if self.task_state != 999:
                    self.get_logger().info("TÜM GÖREVLER TAMAMLANDI.")
                    self.task_state = 999
                return

        task_type = self.current_task['type']

        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'GOTO_WP1':
            self.execute_goto_wp1()
        elif task_type == 'WP2':
            self.execute_turbo_approach()
        elif task_type == 'KAMIKAZE':
            self.execute_kamikaze()
        elif task_type == 'STABILIZE':
            self.execute_stabilize()
        elif task_type == 'RTL':
            self.execute_rtl()
        elif task_type == 'LAND':
            self.execute_land()



    def execute_takeoff(self):
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
                self.get_logger().info(f"   İrtifa: {self.current_pos['z']:.1f}m / {target_alt}m")
            self.log_counter += 1

            if self.current_pos['z'] > (target_alt - 5.0):
                self.finish_task()


    def execute_goto_wp1(self):
 
        if self.task_state == 0:
            self.get_logger().info(f">> GOTO_WP1: ({self.wp1_x:.0f}, {self.wp1_y:.0f})")
            
            # Önce mevcut konumu setpoint olarak gönder (OFFBOARD için gerekli)
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z'])
            
            self.set_mode_command("OFFBOARD")
            self.task_state = 1

        elif self.task_state == 1:
            self.publish_setpoint(self.wp1_x, self.wp1_y, WP1_ALT)
            dist = self.distance_to(self.wp1_x, self.wp1_y)

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   WP1'e mesafe: {dist:.0f}m")
            self.log_counter += 1

            if dist < 50.0:
                self.get_logger().info(">> WP1'E ULAŞILDI")
                self.finish_task()


    def execute_turbo_approach(self):

        overshoot_x = self.qr_x + (self.qr_x - self.wp1_x) * 1.0
        overshoot_y = self.qr_y + (self.qr_y - self.wp1_y) * 1.0

        self.publish_setpoint(overshoot_x, overshoot_y, WP1_ALT)
        dist = self.distance_to(self.qr_x, self.qr_y)

        if self.log_counter % 20 == 0:
            self.get_logger().info(f"   TURBO APPROACH | QR'a mesafe: {dist:.0f}m | Sınır: {WP2_DISTANCE}m")
        self.log_counter += 1

        if dist < WP2_DISTANCE:
            self.get_logger().info(">> 200M SINIRI GEÇİLDİ! KAMİKAZE BAŞLIYOR")
            self.finish_task()


    def execute_kamikaze(self):
 
        yaw = self.yaw_to(self.qr_x, self.qr_y)
        dist = self.distance_to(self.qr_x, self.qr_y)

        if self.task_state == 0:  # Phase 1: Yumuşak dalış
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_1, yaw=yaw, thrust=DIVE_THRUST)

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 1 (+{DIVE_PITCH_1}°) | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < 130:
                self.get_logger().info(">> DIVE Phase 2'ye geçiş")
                self.task_state = 1

        elif self.task_state == 1:  # Phase 2: Orta dalış
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_2, yaw=yaw, thrust=DIVE_THRUST)

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 2 (+{DIVE_PITCH_2}°) | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < 110:
                self.get_logger().info(">> DIVE Phase 3'e geçiş")
                self.task_state = 2

        elif self.task_state == 2:  # Phase 3: Dik dalış
            self.publish_attitude(roll=0.0, pitch=DIVE_PITCH_3, yaw=yaw, thrust=DIVE_THRUST)

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   DIVE Phase 3 (+{DIVE_PITCH_3}°) | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < (PULL_UP_ALT + 5):
                self.get_logger().info(f">> {PULL_UP_ALT}m İRTİFASINA ULAŞILDI - PULL-UP BAŞLIYOR")
                self.task_state = 3

        elif self.task_state == 3:  # Pull-up Phase 1 (60-70m)
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_1, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 1 | Pitch: {PULLUP_PITCH_1}° Roll: {PULLUP_ROLL}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 70.0:
                self.task_state = 4

        elif self.task_state == 4:  # Pull-up Phase 2 (70-80m)
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_2, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 2 | Pitch: {PULLUP_PITCH_2}° Roll: {PULLUP_ROLL}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 80.0:
                self.task_state = 5

        elif self.task_state == 5:  # Pull-up Phase 3 (80-90m)
            self.publish_attitude(roll=PULLUP_ROLL, pitch=PULLUP_PITCH_3, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 3 | Pitch: {PULLUP_PITCH_3}° Roll: {PULLUP_ROLL}° | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > 90.0:
                self.task_state = 6

        elif self.task_state == 6:  # Pull-up Phase 4 (90m+ düz uçuş)
            self.publish_attitude(roll=0.0, pitch=PULLUP_PITCH_4, yaw=yaw, thrust=PULLUP_THRUST)
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   PULL-UP Phase 4 | Düz uçuş | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1
            if self.current_pos['z'] > SAFE_ALT:
                self.get_logger().info(">> GÜVENLİ İRTİFAYA ULAŞILDI - KAMİKAZE TAMAMLANDI")
                self.finish_task()


    def execute_stabilize(self):
        """
        Kamikaze sonrası stabilizasyon - Position setpoint göndererek 
        PX4'ün attitude modundan düzgün çıkmasını sağlar
        """
        if self.task_state == 0:
            self.get_logger().info(">> STABILIZE: Position moduna geçiliyor")
            # Mevcut konumu kaydet
            self.stabilize_x = self.current_pos['x']
            self.stabilize_y = self.current_pos['y']
            self.task_state = 1
            self.timeout_counter = 0

        elif self.task_state == 1:
            # Position setpoint gönder (3 saniye = 60 loop @ 50ms)
            self.publish_setpoint(self.stabilize_x, self.stabilize_y, SAFE_ALT)
            self.timeout_counter += 1
            
            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   STABILIZE | İrtifa: {self.current_pos['z']:.0f}m | {self.timeout_counter}/120")
            self.log_counter += 1
            
            if self.timeout_counter >= 120:  # 6 saniye
                self.get_logger().info(">> STABİLİZASYON TAMAMLANDI - RTL'E GEÇİLİYOR")
                self.finish_task()


    def execute_rtl(self):

        if self.task_state == 0:
            self.get_logger().info(">> RTL: Eve dönülüyor")
            self.set_mode_command("AUTO.RTL")
            self.task_state = 1

        elif self.task_state == 1:
            # RTL tamamlandığında home'a yaklaşmayı bekle
            home_dist = self.distance_to(0.0, 0.0)  # Home = origin
            if self.log_counter % 40 == 0:
                self.get_logger().info(f"   RTL | Home'a mesafe: {home_dist:.0f}m | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            # Home'a yaklaştığında ve alçaldığında LAND'a geç
            if home_dist < 50.0 and self.current_pos['z'] < 80.0:
                self.get_logger().info(">> RTL TAMAMLANDI - LAND'A GEÇİLİYOR")
                self.finish_task()


    def execute_land(self):

        if self.task_state == 0:
            self.get_logger().info(">> LAND: İniş yapılıyor")
            self.set_mode_command("AUTO.LAND")
            self.task_state = 1

        elif self.task_state == 1:
            if self.log_counter % 40 == 0:
                self.get_logger().info(f"   LAND | İrtifa: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            # Yere indiğinde görevi tamamla
            if self.current_pos['z'] < 2.0:
                self.get_logger().info(">> İNİŞ TAMAMLANDI")
                self.finish_task()


    def finish_task(self):
        """Mevcut görevi tamamlandı olarak işaretle."""
        self.get_logger().info(f"✓ GÖREV TAMAMLANDI: {self.current_task['type']}")
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0
        self.timeout_counter = 0


    def distance_to(self, x, y):
        """2D mesafe hesapla (yatay düzlem)."""
        return math.sqrt((x - self.current_pos['x'])**2 + (y - self.current_pos['y'])**2)


    def yaw_to(self, x, y):
        """Hedef noktaya yaw açısı hesapla (derece)."""
        return math.degrees(math.atan2(y - self.current_pos['y'], x - self.current_pos['x']))


    def global_to_local(self, lat, lon):
        """
        GPS koordinatlarını (lat, lon) local ENU koordinatlarına çevir.
        Returns: (x, y) tuple - Doğu ve Kuzey ofseti (metre)
        """
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean)  # ENU X (Doğu)
        y = d_lat * R                        # ENU Y (Kuzey)
        return x, y


    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Euler açılarından (radyan) quaternion'a dönüşüm.
        Returns: (x, y, z, w) tuple
        """
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


    def publish_attitude(self, roll, pitch, yaw, thrust):
        """
        Attitude setpoint yayınla.
        roll, pitch, yaw: Derece cinsinden
        thrust: 0.0 - 1.0 arası
        """
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        
        msg.type_mask = 7  # IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE

        # Derece -> Radyan dönüşümü
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Quaternion hesapla
        x, y, z, w = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        msg.thrust = float(thrust)
        self.attitude_pub.publish(msg)

    def send_takeoff_command(self, relative_alt):
        """Takeoff komutu gönder (göreceli irtifa)."""
        req = CommandTOL.Request()
        req.altitude = self.home_alt_amsl + relative_alt
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.takeoff_client.call_async(req)


    def send_arm_command(self, arm_status):
        """Arm/Disarm komutu gönder."""
        req = CommandBool.Request()
        req.value = arm_status
        self.arming_client.call_async(req)


    def set_mode_command(self, mode):
        """Mod değiştir komutu gönder."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)


    def state_callback(self, msg):
        """State subscriber callback."""
        self.current_state = msg


    def local_position_callback(self, msg):
        """Local position subscriber callback."""
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z


    def global_position_callback(self, msg):
        """Global position subscriber callback. Home pozisyonunu kaydeder."""
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"HOME POS SET: {msg.latitude:.6f}, {msg.longitude:.6f}")
            self.get_logger().info(f"HOME AMSL: {self.home_alt_amsl:.1f}m")


    def home_position_callback(self, msg):
        """Home position subscriber callback. MAVROS origin'in ayarlandığını belirler."""
        if not self.mavros_origin_set and msg.geo.latitude != 0:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS ORIGIN AYARLANDI")


def main(args=None):
    rclpy.init(args=args)
    node = QR2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
