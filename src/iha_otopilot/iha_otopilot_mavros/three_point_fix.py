#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# PX4_MSGS yerine MAVROS ve ROS standart mesajları
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandLong
import math

class MissionsMavros(Node):

    def __init__(self):
        super().__init__('mission_node_mavros')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- ABONELİKLER (Subscribers) ---
        # VehicleStatus yerine State dinliyoruz
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        
        # GPS verisi (VehicleGlobalPosition yerine NavSatFix)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_pos_callback, qos_profile)

        # --- YAYINCILAR (Publishers) ---
        # OffboardControlMode YOKTUR. MAVROS'ta setpoint göndermek yeterlidir.
        
        # TrajectorySetpoint yerine PoseStamped (Local Position)
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # --- SERVİSLER (VehicleCommand yerine) ---
        # MAVROS'ta komutlar Service ile yapılır
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Servislerin hazır olmasını bekle
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servisler bekleniyor...')

        # --- DEĞİŞKENLER ---
        self.current_state = State()
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0 # MAVROS'ta deniz seviyesi (AMSL) farkı için gerekli
        self.home_set = False

        self.counter = 0
        self.mission_step = 0 
        
        # Senin Waypoint Listen
        self.waypoints = [
            {'lat': 47.399500, 'lon': 8.530000, 'alt': 50.0}, 
            {'lat': 47.398023, 'lon': 8.535433, 'alt': 50.0},  
            {'lat': 47.395327, 'lon': 8.539971, 'alt': 50.0},  
            {'lat': 47.397742, 'lon': 8.545594, 'alt': 50.0}   
        ]
        self.current_wp_index = 0

        # 10Hz Zamanlayıcı
        self.timer = self.create_timer(0.1, self.timer_callback)
        print(" GPS Bekleniyor ")

    def state_callback(self, msg):
        self.current_state = msg

    def global_pos_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        # İlk GPS verisi baslangıç noktası
        if not self.home_set and msg.latitude != 0.0:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.home_set = True
            print(f"Baslangic Konumu: {self.home_lat}, {self.home_lon}")

    def timer_callback(self):
        # GPS Yoksa hiçbir şey yapma
        if not self.home_set:
            return

        # MAVROS için Offboard'a geçmeden önce sürekli veri basmak gerekir.
        # Bu yüzden trajectory fonksiyonunu her zaman hazır tutuyoruz.
        # Henüz görev başlamadıysa, olduğumuz yeri hedef gösterelim (Safety)
        if self.mission_step == 0 and self.counter > 200:
             # Sadece transition anında boş veri basmak için
             pass

        # State machine 
        
        # SIFIRINCI AŞAMA (Hazırlık ve Kalkış)
        if self.mission_step == 0:
            
            # MAVROS için resetleme komutuna gerek yoktur, servisler temizdir.
            
            if self.counter == 20:
                print("TAKEOFF COMMAND SENT")
                # Senin sıralaman: Önce Takeoff emri ver
                self.send_takeoff_command(altitude=50.0)
            
            elif self.counter == 30:
                if not self.current_state.armed:
                    print("ARM COMMAND SENT")
                    # Sonra Motorları Başlat
                    self.send_arm_command(True)
            
            # Kalkışın bitmesini bekliyoruz (Basit zamanlayıcı mantığı)
            # MAVROS'ta global_pos.alt AMSL'dir (Deniz seviyesi). 
            # O yüzden (Şu anki - Ev) > 40 diyerek kontrol ediyoruz.
            relative_alt = self.current_alt - self.home_alt
            
            if relative_alt > 40.0: 
                 # Yeterince yükseldik
                 pass

            # 20 saniye sonra göreve başla (Senin mantığın)
            if self.counter > 200: 
                print("WAYPOINT MISSION STARTING - SWITCHING TO OFFBOARD")
                self.set_mode_command("OFFBOARD")
                self.mission_step = 1

        # WAYPOINT AŞAMASI
        elif self.mission_step == 1:
            target = self.waypoints[self.current_wp_index]
            
            # Senin matematik fonksiyonunu kullanıyoruz
            # Fonksiyon bize NED (Kuzey, Doğu) veriyor.
            ned_x, ned_y = self.global_position_ned_position(target['lat'], target['lon'])
            
            # DİKKAT: Senin kodunda Z negatif (Aşağı). MAVROS'ta Z Pozitif (Yukarı).
            # Senin listende 'alt': 50.0 yazıyor, yani +50 metre istiyorsun.
            target_alt = target['alt'] 

            # Hedefi MAVROS'a gönderme (TrajectorySetpoint yerine PoseStamped)
            self.publish_trajectory_setpoint(ned_x, ned_y, target_alt)

            # Mesafe hesabı
            dist = self.calculate_distance_to_target(ned_x, ned_y)
            
            if dist < 40.0: # Uçak olduğu için toleransı 40m yaptım
                print(f"point {self.current_wp_index + 1} arrived, next point ->")
                self.current_wp_index += 1
                
                if self.current_wp_index >= len(self.waypoints):
                    print("FINISH, RETURN TO LAUNCH (RTL)")
                    self.mission_step = 2

        # BİTİŞ AŞAMASI
        elif self.mission_step == 2:
            if self.current_state.mode != "AUTO.RTL":
                print("RTL COMMAND SENT")
                self.set_mode_command("AUTO.RTL")
            
        self.counter += 1

    # --- SENİN MATEMATİK FONKSİYONLARIN ---

    def global_position_ned_position(self, lat_target, lon_target):
        # Senin yazdığın matematik aynen korunuyor.
        R = 6371000.0 
        
        d_lat = math.radians(lat_target - self.home_lat)
        d_lon = math.radians(lon_target - self.home_lon)
        lateral = math.radians(self.home_lat)

        x = d_lat * R # Kuzey (North)
        y = d_lon * R * math.cos(lateral) # Doğu (East)
        
        return x, y

    def calculate_distance_to_target(self, tx, ty):
        # Mevcut konumu senin matematiğinle metreye çeviriyoruz
        cx, cy = self.global_position_ned_position(self.current_lat, self.current_lon)
        return math.sqrt((tx - cx)**2 + (ty - cy)**2)


    # --- DÖNÜŞTÜRÜLMÜŞ YAYIN FONKSİYONLARI ---
    
    # PX4_MSGS'deki TrajectorySetpoint yerine MAVROS PoseStamped
    def publish_trajectory_setpoint(self, x_ned, y_ned, z_alt):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # ÇOK ÖNEMLİ KOORDİNAT DÖNÜŞÜMÜ 
        # Senin matematiğin NED (North, East) hesaplıyor: x=Kuzey, y=Doğu
        # MAVROS ENU (East, North) istiyor: x=Doğu, y=Kuzey
        
        msg.pose.position.x = y_ned  # MAVROS X (Doğu) <- Senin Y (Doğu)
        msg.pose.position.y = x_ned  # MAVROS Y (Kuzey) <- Senin X (Kuzey)
        msg.pose.position.z = z_alt  # MAVROS Z (Yukarı) <- Pozitif İrtifa
        
        # Uçak olduğu için Orientation'ı (Yönelim) boş bırakıyoruz, 
        # Position Control modunda uçak bir sonraki noktaya burnunu çevirir.
        msg.pose.orientation.w = 1.0

        self.local_pos_pub.publish(msg)

    # --- SERVİS WRAPPERLARI (VehicleCommand yerine) ---

    def send_takeoff_command(self, altitude):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.latitude = 0.0 # 0 = Olduğu yerden
        req.longitude = 0.0
        req.altitude = altitude
        self.takeoff_client.call_async(req)

    def send_arm_command(self, arm_status):
        req = CommandBool.Request()
        req.value = arm_status
        self.arming_client.call_async(req)

    def set_mode_command(self, mode_name):
        req = SetMode.Request()
        req.custom_mode = mode_name
        self.set_mode_client.call_async(req)
        
    def send_land_command(self):
        req = CommandTOL.Request()
        self.land_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MissionsMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()