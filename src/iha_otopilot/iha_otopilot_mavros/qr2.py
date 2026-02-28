#!/usr/bin/env python3
"""
QR2 - Kamikaze Mission State Machine
=====================================

Görev Akışı:
    TAKEOFF → GOTO_QR → KAMIKAZE → RECOVERY → RTL → LAND

Uçuş Mantığı:
    - Uçak her zaman QR noktasına doğru gider (lineer güzergah)
    - QR'a 200m kalana kadar: Normal setpoint
    - 200m - 60m arası: Yavaşla (hala QR hedefli)
    - 60m'de: Kamikaze dalışı başlar (attitude control)
    - Pull-up sonrası: Recovery noktasına git

Çalıştırma:
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

Sen terminale udp://:14540@127.0.0.1:14557 yazdığında, MAVROS düğümüne şunu diyorsun: "Gelen verileri 14540'tan dinle, komutları (senin yazdığın o setpointler, kamikaze dalışları) 127.0.0.1 (kendi bilgisayarın) üzerindeki 14557 numaralı PX4 kapısına yolla."
Senin Kodun: Sen ROS 2'de /mavros/state dinler veya /mavros/setpoint_raw/attitude basarken aslında MAVROS arkada bunları MAVLink paketlerine çevirip işte bu UDP portundan uçağa akıtır.

source /opt/ros/humble/setup.bash
source ~/sartek_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH=/home/mert/sartek_ws/install/sartek_sim/share/sartek_sim/models:/home/mert/sartek_ws/install/sartek_sim/share/sartek_sim/worlds:/home/mert/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH

gz sim -v 4 yunuseli.sdf

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, AttitudeTarget, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
import math


# ═══════════════════════════════════════════════════════════════
# QR KOORDİNATLARI (Hedef)
# ═══════════════════════════════════════════════════════════════
QR_LAT = 40.230712658763466
QR_LON = 29.006760026391138


# ═══════════════════════════════════════════════════════════════
# MİSYON PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
TAKEOFF_ALT = 100.0          # Kalkış irtifası (m)
CRUISE_ALT = 110.0           # Seyir irtifası (m)
RECOVERY_DISTANCE = 150.0    # QR'dan recovery noktası mesafesi (m)

# Mesafe eşikleri
SLOWDOWN_DISTANCE = 300.0    # Bu mesafeden sonra yavaşla
DIVE_DISTANCE = 110.0         # Bu mesafede kamikaze başlar

# Hız Parametreleri (PX4 FW_AIRSPD_MAX dinamik değişim)
CRUISE_AIRSPEED = 25.0       # Normal seyir hızı (m/s)
SLOWDOWN_AIRSPEED = 14.0     # Yavaşlama hızı (m/s)

# Kamikaze Parametreleri
# publish attitude içinde +pitch = burun aşağı, -pitch = burun yukarı çünkü base_link frame kullanılıyor base_link'te uçağın burnu x ekseninde, sağ kanadı y ekseninde, aşağısı z ekseninde olur


DIVE_ANGLE = 45.0            # Dalış açısı (derece)
PULLUP_ALT = 50.0            # Pull-up başlama irtifası (m)
DIVE_THRUST = 0.1
PULLUP_THRUST = 0.8


class QR2(Node):

    def __init__(self):
        super().__init__('qr2_node')

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
        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, qos_profile)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(ParamSetV2, '/mavros/param/set')

        # State Variables
        self.current_state = State()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_speed = 0.0
        self.home_pos = None
        self.mavros_origin_set = False
        self.home_alt_amsl = 0.0

        # Mission Variables
        self.mission_queue = []
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0
        self.current_target_speed = 0.0  # Anlık hedef seyir hızımız

        # Waypoint Variables
        self.qr_x = 0.0
        self.qr_y = 0.0
        self.recovery_x = 0.0
        self.recovery_y = 0.0
        
        # Unit vector (home → QR direction)
        self.ux = 0.0
        self.uy = 0.0
        
        # Kamikaze için yaw kaydı
        self.dive_yaw = 0.0

        # ═══ Sürekli Setpoint (Timer) Değişkenleri ═══
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0
        self.enable_position_setpoint = False

        # Timer'lar
        self.timer = self.create_timer(0.05, self.control_loop)
        self.setpoint_timer = self.create_timer(0.1, self.timer_setpoint_callback)
        self.get_logger().info("QR2 State Machine başlatıldı")

    def timer_setpoint_callback(self):
        """10 Hz'de arka planda sürekli setpoint gönderir (Offboard ve zamanlama hatalarını önler)."""
        if self.enable_position_setpoint:
            self.publish_setpoint(self.current_target_x, self.current_target_y, self.current_target_z)


    # ═══════════════════════════════════════════════════════════════
    # MISSION SETUP
    # ═══════════════════════════════════════════════════════════════
    def setup_mission(self):
        """Görev noktalarını hesapla ve mission queue'yu oluştur."""
        if not self.home_pos:
            return

        # QR koordinatlarını local'e çevir
        self.qr_x, self.qr_y = self.global_to_local(QR_LAT, QR_LON)
        
        # Birim vektör: Home → QR yönünde
        dx = self.qr_x
        dy = self.qr_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist > 0:
            self.ux, self.uy = dx / dist, dy / dist
        else:
            self.ux, self.uy = 1.0, 0.0

        # Recovery Point: QR'dan geriye doğru
        self.recovery_x = self.qr_x - self.ux * RECOVERY_DISTANCE
        self.recovery_y = self.qr_y - self.uy * RECOVERY_DISTANCE

        self.get_logger().info(f"═══ GÖREV NOKTALARI ═══")
        self.get_logger().info(f"  QR:       ({self.qr_x:.0f}, {self.qr_y:.0f})")
        self.get_logger().info(f"  RECOVERY: ({self.recovery_x:.0f}, {self.recovery_y:.0f})")
        self.get_logger().info(f"  Slowdown: {SLOWDOWN_DISTANCE}m | Dive: {DIVE_DISTANCE}m")

        # Mission Queue
        self.mission_queue = [
            {'type': 'TAKEOFF', 'alt': TAKEOFF_ALT},
            {'type': 'GOTO_QR'},            # QR'a doğru git, mesafeye göre davran
            {'type': 'KAMIKAZE'},           # 45° dalış + pull-up
            {'type': 'RECOVERY'},           # Recovery noktasına git
            {'type': 'RTL'},
            {'type': 'LAND'}
        ]


    # ═══════════════════════════════════════════════════════════════
    # CONTROL LOOP
    # ═══════════════════════════════════════════════════════════════
    def control_loop(self):
        """50ms'de bir çalışan ana kontrol döngüsü."""
        if not self.home_pos or not self.mavros_origin_set:
            return

        # İlk çalışmada mission'ı kur
        if len(self.mission_queue) == 0 and self.current_task is None and self.task_state == 0:
            self.setup_mission()

        # Yeni göreve geç
        if self.current_task is None:
            if len(self.mission_queue) > 0:
                self.current_task = self.mission_queue.pop(0)
                self.task_state = 0
                self.log_counter = 0
                self.get_logger().info(f"\n>> YENİ GÖREV: {self.current_task['type']}")
            else:
                if self.task_state != 999:
                    self.get_logger().info("═══ TÜM GÖREVLER TAMAMLANDI ═══")
                    self.task_state = 999
                return

        # Görev yönlendirme
        task_type = self.current_task['type']
        
        if task_type == 'TAKEOFF':
            self.execute_takeoff()
        elif task_type == 'GOTO_QR':
            self.execute_goto_qr()
        elif task_type == 'KAMIKAZE':
            self.execute_kamikaze()
        elif task_type == 'RECOVERY':
            self.execute_recovery()
        elif task_type == 'RTL':
            self.execute_rtl()
        elif task_type == 'LAND':
            self.execute_land()


    # ═══════════════════════════════════════════════════════════════
    # GÖREV FONKSİYONLARI
    # ═══════════════════════════════════════════════════════════════

    def execute_takeoff(self):
        """Kalkış görevi."""
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


    def execute_goto_qr(self):
        """
        QR noktasına git - hedef HER ZAMAN QR.
        Mesafeye göre fazlar:
            > 300m: Normal seyir (setpoint)
            300m - 110m: Yavaşlama (FW_AIRSPD_MAX düşürülür)
            < 110m: Kamikaze tetikle
        """
        if self.task_state == 0:
            self.get_logger().info(f">> GOTO_QR: Hedef ({self.qr_x:.0f}, {self.qr_y:.0f})")
            
            # Hedefi güncelle. Timer arkada bu hedefe sürekli (10 Hz) basmaya devam eder!
            self.current_target_x = self.qr_x
            self.current_target_y = self.qr_y
            self.current_target_z = CRUISE_ALT
            self.enable_position_setpoint = True
            
            # Timer zaten çalıştığı için beklemeden OFFBOARD'a geçebiliriz
            self.set_mode_command("OFFBOARD")
            self.task_state = 1

        elif self.task_state == 1:
            dist_to_qr = self.distance_to(self.qr_x, self.qr_y)
            
            # Hedefi sürekli güncelle (GOTO_QR'da hedef sabit zaten)
            self.current_target_x = self.qr_x
            self.current_target_y = self.qr_y
            self.current_target_z = CRUISE_ALT
            
            # ═══ DİNAMİK HIZ LİMİTİ (SAFKOD) ═══
            if dist_to_qr > SLOWDOWN_DISTANCE:
                self.set_desired_speed(CRUISE_AIRSPEED)
                phase = "CRUISE"
            elif dist_to_qr > DIVE_DISTANCE:
                self.set_desired_speed(SLOWDOWN_AIRSPEED)
                phase = "SLOWDOWN"
            else:
                phase = "DIVE_TRIGGER"

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   {phase} | QR: {dist_to_qr:.0f}m | Hız: {self.current_speed:.1f} m/s | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            # KAMIKAZE TETİKLEME
            if dist_to_qr <= DIVE_DISTANCE:
                self.get_logger().info(f">> KAMİKAZE TETİKLENDİ (QR: {dist_to_qr:.0f}m)")
                self.enable_position_setpoint = False  # Kamikaze'de attitude kullanılacağı için position setpoint'i durdur
                self.finish_task()


    def execute_kamikaze(self):
        """
        Kamikaze dalış manevrası.
        ATTITUDE CONTROL - PX4 müdahalesi yok!
        Lateral düzeltme ile QR kamera merkezinde tutulur.
        """
        # ═══ LATERAL DÜZELTME PARAMETRELERİ ═══
        ROLL_GAIN = 2.0          # Yatay hata başına roll (derece/metre)
        MAX_CORRECTION_ROLL = 15.0   # Maksimum düzeltme roll açısı
        
        yaw = self.yaw_to(self.qr_x, self.qr_y)
        alt = self.current_pos['z']
        dist_to_qr = self.distance_to(self.qr_x, self.qr_y)
        
        # Yatay sapma hesaplama (cross-track error)
        # QR'a olan vektör
        dx = self.qr_x - self.current_pos['x']
        dy = self.qr_y - self.current_pos['y']
        
        # İdeal yön (unit vector)
        # Uçağın mevcut yaw açısına göre sağ/sol sapma
        yaw_rad = math.radians(yaw)
        
        # Cross product ile yatay sapma (+ = sağda, - = solda)
        # lateral_error = dx * sin(yaw) - dy * cos(yaw) -- bu QR sağda mı solda mı gösterir
        lateral_error = dx * math.sin(yaw_rad) - dy * math.cos(yaw_rad)
        
        # Roll düzeltmesi (QR solda ise sağa yat, sağda ise sola yat)
        correction_roll = -ROLL_GAIN * lateral_error
        correction_roll = max(-MAX_CORRECTION_ROLL, min(MAX_CORRECTION_ROLL, correction_roll))

        # ═══ DIVE FAZI ═══
        if self.task_state == 0:
            self.get_logger().info(f">> KAMİKAZE BAŞLIYOR - {DIVE_ANGLE}° dalış")
            self.task_state = 1

        elif self.task_state == 1:  # DIVE
            # +pitch = burun aşağı, roll = lateral düzeltme
            self.publish_attitude(roll=correction_roll, pitch=DIVE_ANGLE, yaw=yaw, thrust=DIVE_THRUST)
            
            if self.log_counter % 10 == 0:
                self.get_logger().info(f"   DIVE | Pitch: +{DIVE_ANGLE}° | Roll: {correction_roll:+.1f}° | Alt: {alt:.0f}m | QR: {dist_to_qr:.0f}m | Lateral: {lateral_error:+.1f}m")
            self.log_counter += 1
            
            if alt < PULLUP_ALT:
                # Dalış anındaki yaw'ı kaydet (düz pull-up için)
                self.dive_yaw = yaw
                self.get_logger().info(f">> PULL-UP BAŞLIYOR ({PULLUP_ALT}m)")
                self.task_state = 2

        # ═══ PULL-UP FAZI 1: DÜZ ÇIKIŞ ═══
        elif self.task_state == 2:
            # Roll=0, Yaw sabit (dalış anındaki yaw), sadece pitch yukarı
            self.publish_attitude(roll=0.0, pitch=-30.0, yaw=self.dive_yaw, thrust=PULLUP_THRUST)
            
            if self.log_counter % 10 == 0:
                self.get_logger().info(f"   PULL-UP DÜZ | Pitch: -30° | Roll: 0° | Alt: {alt:.0f}m")
            self.log_counter += 1
            
            if alt > 70.0:
                self.get_logger().info(">> PULL-UP FAZI 2: DÖNÜŞ")
                self.task_state = 3

        # ═══ PULL-UP FAZI 2: YUMUŞAK ÇIKIŞ ═══
        elif self.task_state == 3:
            # Hala düz, daha yumuşak pitch
            self.publish_attitude(roll=0.0, pitch=-15.0, yaw=self.dive_yaw, thrust=PULLUP_THRUST)
            
            if self.log_counter % 10 == 0:
                self.get_logger().info(f"   PULL-UP YUMUŞAK | Pitch: -15° | Alt: {alt:.0f}m")
            self.log_counter += 1
            
            if alt > 85.0:
                self.get_logger().info(">> KAMİKAZE TAMAMLANDI - Recovery'ye dönülüyor")
                self.finish_task()


    def execute_recovery(self):
        """Recovery noktasına git (position setpoint)."""
        SAFE_ALT = 90.0
        ARRIVAL_DISTANCE = 50.0

        if self.task_state == 0:
            self.get_logger().info(f">> RECOVERY: ({self.recovery_x:.0f}, {self.recovery_y:.0f})")
            # Hız limitini normale döndür
            self.set_desired_speed(CRUISE_AIRSPEED)
            
            # Timer hedefini ayarla ve position setpoint'i tekrar başlat
            self.current_target_x = self.recovery_x
            self.current_target_y = self.recovery_y
            self.current_target_z = SAFE_ALT
            self.enable_position_setpoint = True
            
            self.task_state = 1

        elif self.task_state == 1:
            dist = self.distance_to(self.recovery_x, self.recovery_y)
            
            # Hedefi sürekli güncelle
            self.current_target_x = self.recovery_x
            self.current_target_y = self.recovery_y
            self.current_target_z = SAFE_ALT

            if self.log_counter % 20 == 0:
                self.get_logger().info(f"   RECOVERY | Mesafe: {dist:.0f}m | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if dist < ARRIVAL_DISTANCE:
                self.get_logger().info(">> RECOVERY TAMAMLANDI")
                self.finish_task()


    def execute_rtl(self):
        """Return to Launch."""
        if self.task_state == 0:
            self.get_logger().info(">> RTL: Eve dönülüyor")
            self.enable_position_setpoint = False  # Artık OFFBOARD kontrolünde değiliz
            self.set_mode_command("AUTO.RTL")
            self.task_state = 1

        elif self.task_state == 1:
            home_dist = self.distance_to(0.0, 0.0)
            
            if self.log_counter % 40 == 0:
                self.get_logger().info(f"   RTL | Home: {home_dist:.0f}m | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if home_dist < 50.0 and self.current_pos['z'] < 80.0:
                self.get_logger().info(">> RTL TAMAMLANDI")
                self.finish_task()


    def execute_land(self):
        """İniş."""
        if self.task_state == 0:
            self.get_logger().info(">> LAND: İniş yapılıyor")
            self.set_mode_command("AUTO.LAND")
            self.task_state = 1

        elif self.task_state == 1:
            if self.log_counter % 40 == 0:
                self.get_logger().info(f"   LAND | Alt: {self.current_pos['z']:.0f}m")
            self.log_counter += 1

            if self.current_pos['z'] < 2.0:
                self.get_logger().info(">> İNİŞ TAMAMLANDI")
                self.finish_task()


    def finish_task(self):
        """Mevcut görevi tamamla."""
        self.get_logger().info(f"✓ {self.current_task['type']} tamamlandı")
        self.current_task = None
        self.task_state = 0
        self.log_counter = 0


    # ═══════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ═══════════════════════════════════════════════════════════════

    def distance_to(self, x, y):
        """2D mesafe hesapla."""
        return math.sqrt((x - self.current_pos['x'])**2 + (y - self.current_pos['y'])**2)

    def yaw_to(self, x, y):
        """Hedef noktaya yaw açısı (derece)."""
        return math.degrees(math.atan2(y - self.current_pos['y'], x - self.current_pos['x']))

    def global_to_local(self, lat, lon):
        """GPS koordinatlarını local ENU'ya çevir."""
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean)
        y = d_lat * R
        return x, y

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Euler (radyan) → Quaternion dönüşümü."""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w


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

    def publish_attitude(self, roll, pitch, yaw, thrust):
        """
        Attitude setpoint gönder.
        roll, pitch, yaw: Derece
        thrust: 0.0 - 1.0
        
        base_link frame:
            +pitch = burun aşağı (dalış)
            -pitch = burun yukarı (tırmanma)
        """
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.type_mask = 7  # IGNORE rates
        
        x, y, z, w = self.euler_to_quaternion(
            math.radians(roll), 
            math.radians(pitch), 
            math.radians(yaw)
        )
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        msg.thrust = float(thrust)
        
        self.attitude_pub.publish(msg)


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
        İstenilen hıza göre PX4'ün o saniyede gereken throttle'ı ayarlamasını sağlayan Ana Fonksiyon.
        Dışarıdan sadece 'self.set_desired_speed(16.0)' yazılır, geri kalan tüm kontrolleri ve
        parametre güvenliğini bu fonksiyon kendisi yönetir. 
        """
        # Eğer zaten istediğimiz hızdaysak kodu yorma, PX4'e sürekli tekrar eden veri gönderme
        if self.current_target_speed == target_speed:
            return
            
        is_slowing_down = target_speed < self.current_target_speed
        self.current_target_speed = target_speed
        
        # Seyir hızı ile trim hızı dengesi
        trim_speed = target_speed - 1.0
        
        # PX4 aerodinamik kuralı: Yavaşlıyorsan önce TRIM düşür, hızlanıyorsan önce MAX yükselt
        if is_slowing_down or target_speed == 0.0:
            self._set_param("FW_AIRSPD_TRIM", trim_speed)
            self._set_param("FW_AIRSPD_MAX", target_speed)
        else:
            self._set_param("FW_AIRSPD_MAX", target_speed)
            self._set_param("FW_AIRSPD_TRIM", trim_speed)
            
        self.get_logger().info(f">> HIZ KONTROL: Uçak hedef seyir hızı {target_speed} m/s olarak ayarlandı.")

    def _set_param(self, param_id, value):
        """Tek bir PX4 parametresini ayarla."""
        req = ParamSetV2.Request()
        req.param_id = param_id
        req.value = ParameterValue()
        req.value.type = ParameterType.PARAMETER_DOUBLE
        req.value.double_value = float(value)
        self.param_set_client.call_async(req)


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
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"HOME: {msg.latitude:.6f}, {msg.longitude:.6f}")
            self.get_logger().info(f"HOME AMSL: {self.home_alt_amsl:.1f}m")

    def home_position_callback(self, msg):
        if not self.mavros_origin_set and msg.geo.latitude != 0:
            self.mavros_origin_set = True
            self.get_logger().info("MAVROS ORIGIN SET")

    def velocity_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        self.current_speed = math.sqrt(vx**2 + vy**2 + vz**2)


def main(args=None):
    rclpy.init(args=args)
    node = QR2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
