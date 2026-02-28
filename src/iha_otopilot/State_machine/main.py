#!/usr/bin/env python3
"""


"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import SetMode
from enum import Enum
import math

# Görev modüllerini import et
from qr_mission import QRMission, MissionStatus


class State(Enum):
    BASLANGIC = 0
    KALKIS = 1
    SEYIR = 2
    ARAMA = 3
    TAKIP = 4
    QR_MISSION = 5
    EVE_DONUS = 6
    ACIL_INIS = 112


class StateMachineNode(Node):
    
    def __init__(self):
        super().__init__('state_machine_node')
        
        # QoS profili
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # aboneler 
        self.state_sub = self.create_subscription(
            MavrosState, '/mavros/state', self._state_callback, qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._global_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._local_callback, qos_profile)
        
        # yayıncılar 
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        
        #client 
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.current_state = State.BASLANGIC
        self.mavros_state = MavrosState()
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.home_pos = None
        
        # Görev parametreleri
        self.hedef_irtifa = 5.0
        self.gorev_tamamlandi = False
        
        
        # QR Mission modülünü oluştur (self referansı ile)
        self.qr_mission = QRMission(self)
        
        # Timer - Ana kontrol döngüsü (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('State Machine başladi.')
    
    # bu ana döngü 50 ms de çağrılıyor
    def control_loop(self):
        
        # Home pozisyonu bekle
        if not self.home_pos:
            return
        
        # Güvenlik kontrolü
        if self._check_emergency():
            self.current_state = State.ACIL_INIS
        
        # Durum makinesi
        if self.current_state == State.BASLANGIC:
            self._state_baslangic()
        elif self.current_state == State.KALKIS:
            self._state_kalkis()
        elif self.current_state == State.SEYIR:
            self._state_seyir()
        elif self.current_state == State.ARAMA:
            self._state_arama()
        elif self.current_state == State.TAKIP:
            self._state_takip()
        elif self.current_state == State.QR_MISSION:
            self._state_qr_mission()  # <-- Modül burada çağrılıyor
        elif self.current_state == State.EVE_DONUS:
            self._state_eve_donus()
        elif self.current_state == State.ACIL_INIS:
            self._state_acil_inis()
    
    # ==================== DURUM FONKSİYONLARI ====================
    def _state_baslangic(self):
        """BAŞLANGIÇ durumu - Sistem kontrolleri."""
        if self._systems_ready():
            self._transition_to(State.KALKIS)
    
    def _state_kalkis(self):
        """KALKIŞ durumu - Hedef irtifaya yükselme."""
        # TODO: Yükselme komutu gönder
        
        if self._get_altitude() >= self.hedef_irtifa:
            self._transition_to(State.SEYIR)
    
    def _state_seyir(self):
        """SEYİR durumu - Hedef noktaya git."""
        # TODO: Waypoint takibi
        
        if self._reached_target():
            self._transition_to(State.ARAMA)
    
    def _state_arama(self):
        """ARAMA durumu - Hedef tarama."""
        # TODO: Arama paterni uç
        
        if self._target_detected():
            self._transition_to(State.TAKIP)
    
    def _state_takip(self):
        """TAKİP durumu - Hedefi takip et."""
        # TODO: Görüntü işleme ile takip
        
        if self._target_locked():
            self.qr_mission.configure(
                target_lat=40.230712658763466,
                target_lon=29.006760026391138,
                pull_up_alt=60.0
            )
            self._transition_to(State.QR_MISSION)
        elif not self._target_detected():
            self._transition_to(State.ARAMA)
    
    def _state_qr_mission(self):
        """
        QR MISSION durumu.
        Bu fonksiyon sadece görev modülünü çağırır ve sonucunu kontrol eder.
        Tüm mantık qr_mission.py içinde!
        """
        status = self.qr_mission.execute()
        
        if status == MissionStatus.COMPLETED:
            self.gorev_tamamlandi = True
            self._transition_to(State.EVE_DONUS)
        elif status == MissionStatus.FAILED:
            self.get_logger().error("QR Mission başarısız!")
            self._transition_to(State.ACIL_INIS)
        # RUNNING durumunda bir şey yapmaya gerek yok, döngü devam eder
    
    def _state_eve_donus(self):
        """EVE DÖNÜŞ durumu - RTL."""
        # TODO: RTL komutu
        
        if self._reached_home():
            self._transition_to(State.BASLANGIC)
    
    def _state_acil_inis(self):
        """ACİL İNİŞ durumu - Derhal in."""
        self.get_logger().warn('ACİL İNİŞ!')
        # TODO: İniş komutu
    
    # ==================== DURUM GEÇİŞ FONKSİYONU ====================
    def _transition_to(self, new_state: State):
        """Durum geçişi için yardımcı fonksiyon."""
        self.get_logger().info(f'Geçiş: {self.current_state.name} -> {new_state.name}')
        self.current_state = new_state
    
    # ==================== ROS2 CALLBACKS ====================
    def _state_callback(self, msg):
        self.mavros_state = msg
    
    def _global_callback(self, msg):
        if not self.home_pos and msg.latitude != 0:
            self.home_pos = msg
            self.get_logger().info("HOME POS SET")
    
    def _local_callback(self, msg):
        self.current_pos['x'] = msg.pose.position.x
        self.current_pos['y'] = msg.pose.position.y
        self.current_pos['z'] = msg.pose.position.z
    
    # ==================== YAYINLAMA FONKSİYONLARI ====================
    def publish_setpoint(self, x, y, z):
        """Position setpoint yayınla."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.local_pos_pub.publish(msg)
    
    def publish_attitude(self, roll, pitch, yaw, thrust):
        """Attitude setpoint yayınla (derece cinsinden)."""
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | 
                         AttitudeTarget.IGNORE_PITCH_RATE | 
                         AttitudeTarget.IGNORE_YAW_RATE)
        
        # Quaternion hesapla
        x, y, z, w = self._euler_to_quaternion(
            math.radians(roll), math.radians(pitch), math.radians(yaw))
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        msg.thrust = thrust
        
        self.attitude_pub.publish(msg)
    
    def set_mode_command(self, mode):
        """Mod değiştir."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
    
    def global_to_local(self, lat, lon):
        """Global koordinatları local'e çevir."""
        R = 6371000.0
        d_lat = math.radians(lat - self.home_pos.latitude)
        d_lon = math.radians(lon - self.home_pos.longitude)
        lat_mean = math.radians(self.home_pos.latitude)
        x = d_lon * R * math.cos(lat_mean)
        y = d_lat * R
        return x, y
    
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Euler -> Quaternion dönüşümü."""
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w
    
    # ==================== YARDIMCI FONKSİYONLAR (PLACEHOLDER) ====================
    def _check_emergency(self) -> bool:
        return False
    
    def _systems_ready(self) -> bool:
        return True
    
    def _get_altitude(self) -> float:
        return self.current_pos['z']
    
    def _reached_target(self) -> bool:
        return False
    
    def _target_detected(self) -> bool:
        return False
    
    def _target_locked(self) -> bool:
        return False
    
    def _reached_home(self) -> bool:
        return False


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()