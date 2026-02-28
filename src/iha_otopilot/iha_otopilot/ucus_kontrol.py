#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import math

class RobustMission(Node):

    def __init__(self):
        super().__init__('hybrid_mission_node')

        # --- QoS Ayarları ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Bağlantılar ---
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

        self.offboard_ctrl_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.traj_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # --- Değişkenler ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_height = 0.0
        
        self.counter = 0
        self.mission_started = False
        self.theta = 0.0

        # Zamanlayıcı (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        print("--- SİSTEM HAZIR: Veri Bekleniyor ---")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def position_callback(self, msg):
        # Yüksekliği al (NED'de Z negatiftir, pozitife çeviriyoruz)
        self.current_height = -1.0 * msg.z

    def timer_callback(self):
        # 1. KALP ATIŞI (HEARTBEAT) - SÜREKLİ GİTMELİ
        # Offboard moda geçmesek bile hazırlık olarak gönderiyoruz
        self.publish_offboard_control_mode()

        # 2. HEDEF GÖNDERME (Offboard aktif olunca işe yarar)
        if self.mission_started:
            self.publish_offboard_maneuvers()

        # 3. DURUM MAKİNESİ (Sıralı İşlemler)
        
        # [Adım 1] Reset ve Hazırlık
        if self.counter == 10:
            print(">> Komut: Sistem Resetleniyor...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, -1.0, -1.0)

        # [Adım 2] Kalkış Moduna Geç (Takeoff Mode)
        # Offboard değil, TAKEOFF kullanıyoruz çünkü pist kalkışı zordur.
        elif self.counter == 20:
            print(">> Komut: Otomatik Kalkış Modu (Takeoff) Ayarlanıyor...")
            # Param1: Pitch (15 derece), Param7: İrtifa (50m)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=15.0, param7=50.0)

        # [Adım 3] Motorları Çalıştır (ARM)
        elif self.counter == 30:
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                print(">> Komut: ARM (Motorlar Başlatılıyor)...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # [Adım 4] Havada Offboard'a Geçiş
        # Uçak 40 metreyi geçtiyse artık kontrolü biz alalım.
        if self.current_height > 40.0 and not self.mission_started:
            print(f">> İRTİFA {self.current_height:.1f}m! Offboard Moda Geçiliyor...")
            
            # Önce hedefi gönder (Çakılmasın)
            self.mission_started = True 
            
            # Sonra modu değiştir
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        self.counter += 1

    # --- YARDIMCI FONKSİYONLAR ---

    def publish_offboard_maneuvers(self):
        msg = TrajectorySetpoint()
        
        # Çember Parametreleri
        radius = 100.0
        speed = 20.0
        target_alt = -60.0 # 60 metrede dön

        # Çember Hesabı
        msg.position = [
            radius * math.cos(self.theta),
            radius * math.sin(self.theta),
            target_alt
        ]
        
        # Yaw (Uçağın burnu dönüş yönüne baksın)
        msg.yaw = self.theta + (math.pi / 2) 

        # Hız vektörlerini boş bırakıyoruz, PX4 pozisyona gitmek için hızı ayarlasın.
        # Fixed wing için pozisyon setpoint'i yeterlidir.
        msg.velocity = [float('nan'), float('nan'), float('nan')]

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_setpoint_pub.publish(msg)

        # Açıyı artır
        self.theta += (speed / radius) * 0.1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False # Hız kontrolü değil, Pozisyon kontrolü yapıyoruz
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobustMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()