#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

#  mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix # GPS 
import math

class Missions(Node):

    def __init__(self):
        super().__init__('three_point_mission')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10                # burada son 10 mesajı tut
        )

        # bu state durumunu alır
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, qos_profile)

        # bu gps kısmından emin değilim 
        self.global_pos_sub = self.create_subscription(
            NavSatFix, 'mavros/global_position/global', self.global_pos_callback, qos_profile)

        # burda ENU var east north up var negatif olay yook 
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)


        # mavrosda servisler yardımıyla iletiişim salanır topic yerine burada servisler var ve mutlaka geri dönüt alması gerekir
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.landing_client = self.create_client(CommandTOL, 'mavros/cmd/land')

        self.current_state = State()
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_set = False

        self.mission_step = 0 
        self.counter = 0

        self.waypoints = [
            {'lat': 47.399500, 'lon': 8.530000, 'alt': 50.0}, 
            {'lat': 47.398023, 'lon': 8.535433, 'alt': 50.0},  
            {'lat': 47.395327, 'lon': 8.539971, 'alt': 50.0},  
            {'lat': 47.397742, 'lon': 8.545594, 'alt': 50.0}   
        ]
        self.current_wp_index = 0

        # Servislerin hazır olmasını beklemek için var 
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming servisi')

        # 10Hz Zamanlayıcı
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("waypoint mission")

    def state_callback(self, msg):
        self.current_state = msg

    def global_pos_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        if not self.home_set and msg.latitude != 0.0:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_set = True
            self.get_logger().info(f"startpoint {self.home_lat}, {self.home_lon}")

    def timer_callback(self):
        if not self.home_set:
            return
        
        # MAVROS'ta Offboard'a geçebilmek için ÖNCEDEN bir miktar setpoint (hedef) göndermelisin.
        # Yoksa PX4 "Hedefim yok, nereye gideyim?" diyip Offboard'u reddeder.
        
        # Bu aşamada sürekli bir hedef yayınlamak zorunda 

        if self.mission_step == 0:
            if self.counter < 50:
                self.publish_local_setpoint(0.0, 0.0, 0.0) 

            elif self.counter == 50:
                if self.current_state.mode != "OFFBOARD":
                    self.set_mode("OFFBOARD")
            
            elif self.counter == 70:
                if not self.current_state.armed:
                    self.arm_vehicle(True)

            elif self.counter == 90:
                self.get_logger().info("TAKEOFF")
                
            elif self.counter > 90:
                self.publish_local_setpoint(0.0, 0.0, 50.0)

                if self.counter > 250: # 15-20 sn sonra
                    self.get_logger().info("waypoint mission")
                    self.mission_step = 1


        elif self.mission_step == 1:
            target = self.waypoints[self.current_wp_index]
            
            # MAVROS ENU (East-North-Up) kullanır!
            # Senin hesaplama fonksiyonunu ENU'ya göre çağırmalıyız.
            enu_x, enu_y = self.global_to_local_enu(target['lat'], target['lon'])
            enu_z = target['alt'] # MAVROS'ta YUKARI POZİTİFTİR (+)

            self.publish_local_setpoint(enu_x, enu_y, enu_z)

            # Mesafe Kontrolü
            dist = self.calculate_distance(enu_x, enu_y, enu_z)
            if dist < 20.0:
                self.get_logger().info(f"Nokta {self.current_wp_index + 1} TAMAM")
                self.current_wp_index += 1
                
                if self.current_wp_index >= len(self.waypoints):
                    self.mission_step = 2

        # AŞAMA 2: İNİŞ
        elif self.mission_step == 2:
            if self.current_state.mode != "AUTO.LAND":
                self.get_logger().info("LAND MODUNA GEÇİLİYOR...")
                self.set_mode("AUTO.LAND") # PX4'te Land modu "AUTO.LAND" olarak geçer

        self.counter += 1

    # --- YARDIMCI FONKSİYONLAR ---

    def global_to_local_enu(self, lat_target, lon_target):
        # MAVROS ENU (Doğu-Kuzey-Yukarı) kullanır.
        # Matematik aynıdır ama X ve Y'nin anlamı değişir.
        R = 6371000.0 
        d_lat = math.radians(lat_target - self.home_lat)
        d_lon = math.radians(lon_target - self.home_lon)
        lat_mean = math.radians(self.home_lat)

        # ENU SİSTEMİ:
        # X EKSENİ = DOĞU (EAST)
        # Y EKSENİ = KUZEY (NORTH)
        x_east = d_lon * R * math.cos(lat_mean)
        y_north = d_lat * R 
        
        return x_east, y_north

    def calculate_distance(self, tx, ty, tz):
        # Mevcut konum zaten local/enu formatında gelebilir ama biz GPS'ten hesaplayalım yine
        cx, cy = self.global_to_local_enu(self.current_lat, self.current_lon)
        return math.sqrt((tx - cx)**2 + (ty - cy)**2)

    # --- MAVROS PUBLISHER FONKSİYONU ---
    def publish_local_setpoint(self, x, y, z):
        msg = PoseStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" # Veya "base_link"
        
        # MAVROS standart ROS koordinat sistemi kullanır (ENU)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Yönelim (Quaternion) - Şimdilik boş (0,0,0,1)
        # Eğer uçağın burnunu çevirmek istersen burayı hesaplaman gerekir.
        msg.pose.orientation.w = 1.0 
        
        self.local_pos_pub.publish(msg)

    # --- SERVICE ÇAĞRILARI (MAVROS FARKI) ---
    def arm_vehicle(self, state):
        req = CommandBool.Request()
        req.value = state
        self.arming_client.call_async(req)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = Missions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()