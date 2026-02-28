#!/usr/bin/env python3
"""
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"



cd ~/sartek_ws
source install/setup.bash
ros2 launch sartek_sim qr_mission_sim.launch.py
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, AttitudeTarget, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import math

class QR(Node):

    def __init__(self):
        super().__init__('qr_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_calibration, qos_profile)
        self.global_pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_calibration, qos_profile)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_calibration, qos_profile)
        self.home_sub = self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_position_callback, qos_profile)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.local_vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')


        self.current_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.home_pos = None
        
        # task state mühim bişey mission queue içindeki adımları da if elif else yapısı için numaralndırır ve biri bittiğinde artara kilerler yeni görev geldiğinde state sıfırlanır ve yeniden sırayla başlar
        self.mission_queue = []   # Görev listesi
        self.current_task = None  # Şu an yapılan görev
        self.task_state = 0       # Görevin içindeki aşama
        self.theta = 0.0          # Daire çizmek için açı sayacı
        self.mavros_origin_set = False
        self.mission_steps()

        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("örnek olsun diye yazdım istediğin zaman bu tip gibi mesaj yazdırabilirsin hatta loglama da bu şekilde sanırım ")
        self.get_logger().info("GÖREV YÖNETİCİSİ BAŞLATILDI")

    def mission_steps(self):
        # İstediğin emirleri sırasıyla buraya ekle
        
    
        self.mission_queue.append({'type': 'TAKEOFF', 'alt': 60.0})
        
      
        self.mission_queue.append({'type': 'waypnt', 'lat': 40.230712658763466, 'lon': 29.006760026391138, 'alt': 60.0})
        

        self.mission_queue.append({'type': 'CIRCLE', 'radius': 40.0, 'turns': 1, 'alt': 60.0})

 
        self.mission_queue.append({'type': 'SPIRAL_UP', 'radius': 40.0, 'target_alt': 65.0})

        #SPIRAL_UP sonrası QR noktasından UZAKLAŞMA
        # Yaklaşık 200m kuzeye git (lat +0.0018 ≈ 200m)
        # Bu sayede kamikaze dalışı için yeterli mesafe ve açı kazanılır
        self.mission_queue.append({
            'type': 'UZAKLASMA', 
            'lat': 40.232512658763466,  # QR noktasından ~200m kuzeyde
            'lon': 29.006760026391138, 
            'alt': 200.0  # Kamikaze için yükseklik kazan
        })

        self.mission_queue.append({
            'type': 'KAMIKAZE', 
            'lat': 40.230712658763466, 
            'lon': 29.006760026391138, 
            'dive_speed': 25.0,   
            'final_alt': 0.0      
        })

        self.mission_queue.append({'type': 'RTL'})

    def control_loop(self):
        if not self.home_pos or not self.mavros_origin_set:
            return

        # Eğer şu an bir görev yoksa, listeden yenisini al
        if self.current_task is None:
            if len(self.mission_queue) > 0:
                self.current_task = self.mission_queue.pop(0)
                self.task_state = 0 # Yeni görevi sıfırla
                self.theta = 0.0    # Açıları sıfırla
                self.get_logger().info(f"YENİ GÖREV BAŞLADI: {self.current_task['type']}")
            else:
                self.get_logger().info("TÜM GÖREVLER BİTTİ.")
                self.timer.cancel() # Döngüyü durdur
                return

        # MEVCUT GÖREVİ YÜRÜT
        task_type = self.current_task['type']

        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'waypnt':
            self.execute_waypnt()
        elif task_type == 'CIRCLE':
            self.execute_circle()
        elif task_type == 'SPIRAL_UP':
            self.execute_spiral()
        elif task_type == 'UZAKLASMA':
            self.execute_uzaklasma()
        elif task_type == 'KAMIKAZE':
            self.execute_kamikaze()
        elif task_type == 'RTL':
            self.execute_rtl()


    # bu task_state kullanımıönemli çünkğ kodu dinamşk yapar yani kod whildekai gibi donmaz kontrol eder ve yoluna devam eder

    def execute_takeoff(self):
        if self.task_state == 0:
            self.send_takeoff_command(self.current_task['alt'])
            self.task_state = 1
        
        elif self.task_state == 1:
            if self.current_state.mode == "AUTO.TAKEOFF":
                self.send_arm_command(True)
                self.task_state = 2
        
        elif self.task_state == 2:
            if self.current_pos['z'] > (self.current_task['alt'] - 5.0):
                self.finish_task() # Görev Tamam

    def execute_waypnt(self):
        
        tx, ty = self.global_to_local(self.current_task['lat'], self.current_task['lon'])
        tz = self.current_task['alt']

        if self.task_state == 0:
            self.get_logger().info(f">> waypnt Başladı: Hedefe Gidiliyor...")
            
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z']) 
            
            self.set_mode_command("OFFBOARD")
            
            self.timeout_counter = 0 
            self.task_state = 1

        elif self.task_state == 1:
            self.publish_setpoint(tx, ty, tz)
            
            # Yatay Mesafe Hesabı (Z'yi katmıyoruz, çünkü irtifa farkı yanıltabilir)
            dist = math.sqrt((tx - self.current_pos['x'])**2 + (ty - self.current_pos['y'])**2)
            
            # Hata Ayıklama: Mesafeyi ekrana yazdır (Takılıp takılmadığını gör)
            #kod burada takıloyrud haberin olsun
            if self.timeout_counter % 20 == 0:
                self.get_logger().info(f"Hedefe Mesafe: {dist:.1f}m ")


            if dist < 40.0:
                self.get_logger().info(">> HEDEFE VARILDI ")
                self.finish_task()
                return
            
            # Eğer 60 saniye (1200 döngü) boyunca varamazsa yine de geç.
            self.timeout_counter += 1
            if self.timeout_counter > 1200: 
                self.get_logger().warn(">> ZAMAN AŞIMI! ")
                self.finish_task()

    def execute_circle(self):
        radius = self.current_task['radius']
        alt = self.current_task['alt']
        total_turns = self.current_task['turns']
        

        speed = 15.0 # m/s (tahmini)
        angular_speed = speed / radius 

        # Daire matematiği (Mevcut konum merkezli varsayıyorum basitlik için)
        # Aslında bir merkeze gitmek daha doğrudur ama bu kod "olduğu yerde dön" mantığıdır.
        # Daha düzgün bir daire için merkez ofseti ekliyoruz:
        
        # Daire Merkezi: Uçağın görev başladığı andaki konumu + Yarıçap kadar ileri
        if self.task_state == 0:
            self.center_x = self.current_pos['x'] + radius
            self.center_y = self.current_pos['y']
            self.task_state = 1

        # X = Cx + r * cos(theta)
        # Y = Cy + r * sin(theta)
        next_x = self.center_x + radius * math.cos(self.theta + math.pi) # +pi başlangıç düzeltmesi
        next_y = self.center_y + radius * math.sin(self.theta + math.pi)

        self.publish_setpoint(next_x, next_y, alt)

        # Açıyı artır
        self.theta += angular_speed * 0.05 # dt = 0.05

        # Tur bitti mi? (2 pi * tur sayısı)
        if self.theta > (2 * math.pi * total_turns):
            self.finish_task()

    def execute_spiral(self):
        # Dönerek Yükselme
        radius = self.current_task['radius']
        target_alt = self.current_task['target_alt']
        
        # Merkez belirle (İlk giriş)
        if self.task_state == 0:
            self.center_x = self.current_pos['x'] + radius
            self.center_y = self.current_pos['y']
            self.task_state = 1
        
        # Spiral Matematigi
        next_x = self.center_x + radius * math.cos(self.theta)
        next_y = self.center_y + radius * math.sin(self.theta)
        
        # Yüksekliği yavaşça artır (Her turda 10 metre yüksel gibi)
        # Basitçe: Mevcut yükseklik hedef değilse, hafifçe yukarı setpoint ver
        climb_rate = 5.0 * 0.05 # sn'de 2 metre
        next_z = self.current_pos['z'] + climb_rate
        if next_z > target_alt: next_z = target_alt

        self.publish_setpoint(next_x, next_y, next_z)
        
        self.theta += (15.0 / radius) * 0.05

        if self.current_pos['z'] >= (target_alt - 2.0):
            self.finish_task()

        

    def execute_uzaklasma(self):
        """
        Spiral up sonrası QR noktasından uzaklaşma.
        Kamikaze dalışı için yeterli mesafe ve yükseklik kazanır.
        """
        tx, ty = self.global_to_local(self.current_task['lat'], self.current_task['lon'])
        tz = self.current_task['alt']

        if self.task_state == 0:
            self.get_logger().info(f">> UZAKLAŞMA Başladı: QR'dan uzaklaşılıyor...")
            self.get_logger().info(f"   Hedef irtifa: {tz}m | Kamikaze hazırlığı")
            
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z']) 
            
            self.set_mode_command("OFFBOARD")
            
            self.timeout_counter = 0 
            self.task_state = 1

        elif self.task_state == 1:
            self.publish_setpoint(tx, ty, tz)
            
            # Yatay Mesafe Hesabı
            dist = math.sqrt((tx - self.current_pos['x'])**2 + (ty - self.current_pos['y'])**2)
            
            if self.timeout_counter % 20 == 0:
                self.get_logger().info(f"   Uzaklaşma noktasına: {dist:.1f}m | İrtifa: {self.current_pos['z']:.1f}m")

            if dist < 40.0:
                self.get_logger().info(">> UZAKLAŞMA TAMAMLANDI - KAMİKAZE'YE HAZIR!")
                self.finish_task()
                return
            
            # Timeout kontrolü (60 saniye)
            self.timeout_counter += 1
            if self.timeout_counter > 1200: 
                self.get_logger().warn(">> ZAMAN AŞIMI! ")
                self.finish_task()


    def execute_kamikaze(self):
        """
        Uçağın attitude değerlerini doğrudan kontrol et
        burayı da task_state ile bölerek yapılıyor
Hazırlık 
OFFBOARD 
            2: Hedefe doğru yaklaş (yatay uçuş, position setpoint)
            3: 45 derece dalış (pitch=-45°, roll=0°, yaw=hedef yönü)
            4: Pull-up (pitch=+30°)
            5: Görev tamamlandı
        """
        target_lat = self.current_task['lat']
        target_lon = self.current_task['lon']
        pull_up_alt = 60.0  # Bu irtifada pullup başlayacakyere çakılmayı önle
        
        # Hedefin MAVROS (ENU) koordinatlarını al
        tx, ty = self.global_to_local(target_lat, target_lon)
        
        # Incremental hareket parametreleri (yaklaşma için)
        step_size = 2.0
        

# buradan başlıyor task_state
        if self.task_state == 0:
            self.get_logger().info(">> KAMİKAZE: ")
            self.get_logger().info(f"   Hedef: ({tx:.1f}, {ty:.1f})")
            self.get_logger().info(f"   Pull-up irtifası: {pull_up_alt}m")
            
            # Başlangıç değerlerini kaydet
            self.kamikaze_start_alt = self.current_pos['z']
            
            dx = tx - self.current_pos['x']
            dy = ty - self.current_pos['y']
            self.kamikaze_start_dist = math.sqrt(dx**2 + dy**2)
            # Hedef yönü (yaw açısı) - derece cinsinden
            self.kamikaze_yaw = math.degrees(math.atan2(dy, dx))
            self.kamikaze_angle = math.atan2(dy, dx)  # Radyan (hesaplamalar için)
            
            # Incremental hedef pozisyonu (başlangıçta mevcut konum)
            self.inc_target_x = self.current_pos['x']
            self.inc_target_y = self.current_pos['y']
            self.inc_target_z = self.current_pos['z']
            
            self.get_logger().info(f"   Başlangıç irtifa: {self.kamikaze_start_alt:.1f}m, mesafe: {self.kamikaze_start_dist:.1f}m")
            self.get_logger().info(f"   Hedef yaw: {self.kamikaze_yaw:.1f}°")
            
            # Mevcut konumu setpoint olarak gönder
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z'])
            
            # OFFBOARD moduna geç
            self.set_mode_command("OFFBOARD")
            self.task_state = 1
            self.offboard_counter = 0
            self.kamikaze_log_counter = 0   # bu da terminale log yazdırmak için %20 ile kullanılıyor bu sayede her saniyede bir yazdırıyoor
        
        elif self.task_state == 1:
            self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z'])
            self.offboard_counter += 1
            
            if self.offboard_counter > 20:
                self.get_logger().info(">> OFFBOARD aktif")
                # Incremental hedefi mevcut konuma ayarla
                self.inc_target_x = self.current_pos['x']
                self.inc_target_y = self.current_pos['y']
                self.inc_target_z = self.current_pos['z']
                self.task_state = 2
        
        elif self.task_state == 2:
            dx = tx - self.current_pos['x']
            dy = ty - self.current_pos['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            # Artımlı (incremental) hesap yerine DOĞRUDAN hedefe git diyoruz.
            # Uçak zaten kendi trajectory'sini oluşturur.
            # İrtifayı koru (start_alt), X ve Y hedefine git.
            self.publish_setpoint(tx, ty, self.kamikaze_start_alt)
            
            self.kamikaze_log_counter += 1
            if self.kamikaze_log_counter % 20 == 0:
                self.get_logger().info(f"   Yaklaşma | Mesafe: {dist:.1f}m | İrtifa: {self.current_pos['z']:.1f}m")
            
            # Hedefe 100m kala Attitude (Dalış) kontrolüne geç
            if dist < 100.0:
                self.get_logger().info(">> DALIŞ BAŞLIYOR (ATTITUDE CONTROL) <<")
                self.get_logger().info(f"   Pitch: -45° | Roll: 0° | Yaw: {self.kamikaze_yaw:.1f}°")
                self.dive_start_alt = self.current_pos['z']
                self.task_state = 3
        
        elif self.task_state == 3:
            dx = tx - self.current_pos['x']
            dy = ty - self.current_pos['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            # 🔴 DİNAMİK YAW - Her döngüde güncelle (sabit kalınca sapma yapıyordu)
            dynamic_yaw = math.degrees(math.atan2(dy, dx))
            
            # Attitude setpoint: 
            # - Pitch: -30° (daha yumuşak, -45° çok agresifti ve recovery tetikliyordu)
            # - Roll: 0° (düz kanat)
            # - Yaw: Dinamik hesaplanan hedef yönü
            # - Thrust: 0.3 (minimum kontrol gücü - 0.0 kontrol kaybına sebep oluyordu)
            self.publish_attitude(
                roll=0.0,
                pitch=-30.0,  # -45° yerine -30° (daha güvenli)
                yaw=dynamic_yaw,  # Sabit yaw yerine dinamik
                thrust=0.5  # 0.0 yerine 0.3 (kontrol için gerekli minimum)
            )
            
            self.kamikaze_log_counter += 1
            if self.kamikaze_log_counter % 20 == 0:
                self.get_logger().info(
                    f"   DALIŞ | Mesafe: {dist:.1f}m | "
                    f"İrtifa: {self.current_pos['z']:.1f}m | Yaw: {dynamic_yaw:.1f}°"
                )
            
            # Pull-up irtifasına ulaşıldı mı?
            if self.current_pos['z'] <= (pull_up_alt + 5.0):
                self.get_logger().info(f">> {pull_up_alt}m irtifaya ulaşıldı - PULL-UP!")
                self.task_state = 4
                self.pullup_counter = 0
        
        elif self.task_state == 4:
            # Dinamik yaw hesapla (dalıştan çıkış yönü)
            dx = tx - self.current_pos['x']
            dy = ty - self.current_pos['y']
            dynamic_yaw = math.degrees(math.atan2(dy, dx))
            
            # Attitude setpoint: Pitch=+25° (burun yukarı), Roll=0°
            # Thrust: 0.7 (kontrollü tırmanma - 1.0 çok agresif)
            self.publish_attitude(
                roll=0.0,
                pitch=25.0,  # 30° yerine 25° (daha yumuşak)
                yaw=dynamic_yaw,
                thrust=0.7  # 1.0 yerine 0.7 (daha kontrollü)
            )
            
            self.pullup_counter += 1
            if self.pullup_counter % 20 == 0:
                self.get_logger().info(
                    f"   PULL-UP | İrtifa: {self.current_pos['z']:.1f}m | Pitch: +25°"
                )
            
            # Yeterli irtifa kazanıldı mı? (pull_up_alt + 30m güvenlik marjı)
            if self.current_pos['z'] > (pull_up_alt + 30.0):
                self.get_logger().info(">> KAMIKAZE TAMAMLANDI - Güvenli irtifaya ulaşıldı")
                self.task_state = 5
        
        elif self.task_state == 5:
            self.finish_task()
    

    def publish_velocity(self, vx, vy, vz):
        """Velocity setpoint yayınla referans için kalıyor"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.local_vel_pub.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Euler açılarından (radyan) quaternion'a dönüşüm"""
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

    def publish_attitude(self, roll, pitch, yaw, thrust):
        """
        Attitude setpoint yayınla (roll, pitch, yaw derece cinsinden)
        thrust: 0.0 - 1.0 arası
        """
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Type mask: Sadece orientation ve thrust kullan (body_rate'i yoksay)
        msg.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE
        
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
        
        msg.thrust = thrust
        
        self.attitude_pub.publish(msg)

    def execute_rtl(self):
        if self.task_state == 0:
            self.set_mode_command("AUTO.RTL")
            self.task_state = 1





    def finish_task(self):
        self.get_logger().info(f"GÖREV TAMAMLANDI: {self.current_task['type']}")
        self.current_task = None # Bu görevi boşa çıkar, döngü yenisini alacak

    def publish_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.local_pos_pub.publish(msg)

    def global_to_local(self, lat, lon):
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean) # ENU X (Doğu)
        y = d_lat * R                      # ENU Y (Kuzey)
        return x, y


    def home_position_callback(self, msg):
        if not self.mavros_origin_set and msg.geo.latitude != 0:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS ORIGIN AYARLANDI")





    # Callbackler ve Servisler bunlar bizden kesin istenen şekilde üzerinde oynama yapılmıyor
    # self.state_sub = self.create_subscription(State, '/mavros/state', self.state_calibration, qos_profile)
    def state_calibration(self, msg): self.current_state = msg


    # self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_calibration, qos_profile)
    def local_calibration(self, msg): 
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z

    # başlangıçta hata vermemesi için sıfır da sorguluyorz
    # self.global_pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_calibration, qos_profile)
    def global_calibration(self, msg):
        self.current_amsl = msg.altitude # Anlık AMSL yüksekliğini sürekli güncelle
        
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.home_alt_amsl = msg.altitude # Evin gerçek deniz seviyesi yüksekliğini kaydet
            self.get_logger().info(f"HOME POS SET (AMSL: {self.home_alt_amsl:.1f}m)")


    def send_takeoff_command(self, relative_alt):
        req = CommandTOL.Request()
        # Eğer ev konumu belliyse ona ekle, değilse o anki yüksekliğe ekle
        base_alt = self.home_alt_amsl if hasattr(self, 'home_alt_amsl') else self.current_amsl
        req.altitude = base_alt + relative_alt 
        
        req.latitude = 0.0 # Olduğu yerden kalk
        req.longitude = 0.0
        req.min_pitch = 0.0
        self.takeoff_client.call_async(req)


    def send_arm_command(self, status):
        req = CommandBool.Request()
        req.value = status
        self.arming_client.call_async(req)



    def set_mode_command(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)



def main(args=None):
    rclpy.init(args=args)
    node = QR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()