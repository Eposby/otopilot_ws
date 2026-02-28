#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition
import math

class Missions(Node):

    def __init__(self):
        super().__init__('mission_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # sürekli mesaj
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # son kısım tutulut subscirber geç kalsa bil alır
            history=HistoryPolicy.KEEP_LAST,            # en sonuncuları tutar
            depth=1                 # sadece sonm esaj kalır
        )

        # bu vehicle status durumuna göre değişir
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        
        # gps verisini kontrol eder
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_pos_callback, qos_profile)

        #offboard
        self.offboard_control_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        #waypoint
        self.trajectory_control_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        #vehicle command
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # buraya değişkenleri class içine tanıtmak için yazıldı fonksiyonlarda kullanılacak
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_set = False

        self.counter = 0
        self.mission_step = 0 # waypointler buradan değişiyor
        
        # gps verisi derece cisnsinden uçağa x ve y değerlerini vermek için 
        # x= lateral x yarıçap 
        # y= longtidunal x yarçap x cos(lateral) yapılacak

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

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def global_pos_callback(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt

        # İlk GPS verisi baslangıç noktası
        if not self.home_set and msg.lat != 0.0:
            # ilk sefer çalıştırırken bu çalışır 
            # kod ilk çalıştığında kendisini 0. noktada olduğunu sanabilir böyle olursa kod takılır çalışmaz bunu önlemek için yapılmıştır
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_set = True
            print(f"Baslangic Konumu: {self.home_lat}, {self.home_lon}")

    def timer_callback(self):
        # Heartbeat olumlu mu kontrol et yoksa return yap
        self.publish_offboard_control_mode()

        # GPS Yoksa hiçbir şey yapma
        if not self.home_set:
            return

        # State machine 
        
        # sıfırıncı aşama
        if self.mission_step == 0:
            if self.counter == 10:
                print("sifirlama")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, -1.0, -1.0)
            
            elif self.counter == 20:
                print("TAKEOFF")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=15.0, param7=50.0)
                    # param1 kalkış açısı # param7 yükseklik
            elif self.counter == 30:
                if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    print("ARM")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            

            # global_pos.alt deniz seviyesi olarka hesaplanır
            if self.current_alt > 475.0: # Zürih rakımı 430 olarak al 5 m alttn başlat
                 # Daha güvenli yöntem: Kalkıştan 20 saniye sonra geç
                 pass

            if self.counter > 200: # 20 saniye sonra göreve başla
                print("WAYPOINt MISSION")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard
                        # param1 burada kendi yazılımıımı kullanacağım demek  param2= bu kanallar gibidir 6.0 kanalı offboard kanalına geçer
                self.mission_step = 1

        # WAYPOINT aşaması
        elif self.mission_step == 1:
            target = self.waypoints[self.current_wp_index]
            
            # Lat/Lon'u X/Y/Z (NED)'ye çevir
            ned_x, ned_y = self.global_position_ned_position(target['lat'], target['lon'])
            ned_z = -target['alt'] # Yukarı negatif

            # hedefi px4'e gönderme 
            self.publish_trajectory_setpoint(ned_x, ned_y, ned_z)

            # Hedefe yaklaşınca sabit kanatlı olduğu için dönmeye başlıyor
            dist = self.calculate_distance_to_target(ned_x, ned_y, ned_z)
            
            if dist < 20.0: 
                print(f"point {self.current_wp_index + 1} arrived, next point: {self.current_wp_index + 2}")
                self.current_wp_index += 1
                
                
                if self.current_wp_index >= len(self.waypoints):
                    print("FINISH,  RETURN TO LAUNCH ")
                    self.mission_step = 2


        elif self.mission_step == 2:
            print("LAND")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
        self.counter += 1

    # MATEMATİK FONKSİYONLARI bunları mutlaka sor 

    def global_position_ned_position(self, lat_target, lon_target):
        # maksimm 10 20 km sornasında bozulmalar olabilir 
        
        R = 6371000.0 
        
        d_lat = math.radians(lat_target - self.home_lat)
        d_lon = math.radians(lon_target - self.home_lon)
        lateral = math.radians(self.home_lat)

        x = d_lat * R # Kuzey (North)
        y = d_lon * R * math.cos(lateral) # Doğu (East)
        
        return x, y

    def calculate_distance_to_target(self, tx, ty, tz):
        # Mevcut konumu lateral lontidunal olarak metreye çevir
        cx, cy = self.global_position_ned_position(self.current_lat, self.current_lon)
        return math.sqrt((tx - cx)**2 + (ty - cy)**2)



# alttaki üç fonksiyon px4 içinde zorunlu yapısını değiştiremezsin aynen bu şekilde mesaj dosyalarını içermek zorunda 
# bu kısmı yazarken gerekli developerlari incele

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint() # bu bir boş form içini doldur
        msg.position = [x, y, z]
        # Sabit kanatlı uçak olduğu için, noktaya "Süzülerek" gitmesini istiyoruz.
        # Velocity'i 'nan' yaparsak PX4 en uygun hızı kendi ayarlar.
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_control_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode() # bu bir boş form içini doldur 
        msg.position = True # konumu biz veriyoruz bool tipi var  bbuna true dediğimiz için üstteki trajectory set point kısmındaki position'ı doldurmak zorundayz
        msg.velocity = False # hızı kendi hesaplıyor o yüzden bool false 
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # bu önemli 
        self.offboard_control_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand() # bu da bir form dikkar et 
        msg.command = command
        # burada verieln commandlar yukarıdaki TAKEOFF-LAND commandlarının sayısal halidir biz oraya "VEHICLE_CMD_NAV_LAND" yazıldığında command 21 olur
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1       # alıcı uçağın kendisi eğer birden fazla hava aracı olsa bu değişir ve istediğimiz uçağa yönlendirme olur 
        msg.target_component = 1    # alıcı bölüm otopilot oluyor
        msg.source_system = 1       # gönderen bilgisayar bilgisayar
        msg.source_component = 1    # gönderen bölüm yazılılm 
        msg.from_external = True    # ros2 dışaıdan uçağa müdahale ettiği için bu true oluyor eğer uçağın içinden px4'tn geldiyse bu false olur
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Missions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()