import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition

class FixedWingMission(Node):

    def __init__(self):
        super().__init__('fixed_wing_mission_node')

        # --- DÜZELTME 1: UYUMLU QoS AYARI ---
        # Uçaktan gelen veriyi ne olursa olsun (Reliable veya Best Effort) dinlemek için
        # "Best Effort" ve "Volatile" kombinasyonu en güvenlisidir.
        qos_subscriber = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Komutlar için Reliable (Garantili)
        qos_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setpoint için Best Effort (Hızlı)
        qos_setpoint = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher'lar
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_setpoint)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_setpoint)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_publisher)

        # Subscriber'lar (Yeni QoS ile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_subscriber)
        
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_position_callback, qos_subscriber)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.vehicle_status = VehicleStatus()
        self.home_lat = None
        self.home_lon = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.counter = 0
        self.current_waypoint_index = 0
        self.waypoints = []
        self.acceptance_radius = 20.0 
        self.command_counter = 0
        
        # Veri akışını kontrol etmek için bayrak
        self.status_received = False

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        
        # --- DEBUG: Veri akıyor mu? ---
        if not self.status_received:
            self.get_logger().info(f"BAĞLANTI BAŞARILI: Uçak Durum Verisi Alınıyor! (Mod: {self.nav_state})")
            self.status_received = True

    def global_position_callback(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.get_logger().info(f"GPS KİLİTLENDİ: {self.home_lat}, {self.home_lon}")
            
            # Rota oluştur
            wp1 = self.add_meters_to_gps(self.home_lat, self.home_lon, 150, 0)
            wp2 = self.add_meters_to_gps(self.home_lat, self.home_lon, 150, 100)
            
            self.waypoints = [
                (wp1[0], wp1[1], -50.0),
                (wp2[0], wp2[1], -50.0)
            ]

    def timer_callback(self):
        self.publish_offboard_control_mode()

        # GPS ve Status gelmiyorsa bekle
        if self.home_lat is None or not self.status_received:
            if self.counter % 20 == 0: 
                self.get_logger().info("Veri bekleniyor... (Agent açık mı?)")
            self.counter += 1
            return

        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            self.send_target(target[0], target[1], target[2])
        
        # --- KONTROL MANTIĞI ---
        if self.counter > 20:
            
            # 1. Offboard Kontrolü
            # 14 = OFFBOARD Modu
            if self.nav_state != 14:  
                if self.command_counter % 20 == 0:
                    self.engage_offboard_mode()
                    # Nav State'i ekrana yazdıralım ki ne olduğunu görelim
                    self.get_logger().info(f"Mod Bekleniyor... (Şu anki Mod: {self.nav_state})")
            
            # 2. Arm Kontrolü
            # 2 = ARMED
            elif self.arming_state != 2:
                if self.command_counter % 20 == 0:
                    self.arm()
                    self.get_logger().info("Arm Bekleniyor...")
            
            # 3. Uçuş
            else:
                target = self.waypoints[self.current_waypoint_index]
                dist = self.calculate_dist(target[0], target[1])
                
                if self.counter % 20 == 0:
                    self.get_logger().info(f"UÇUŞTA: Hedef {self.current_waypoint_index+1} Mesafe: {dist:.1f}m")

                if dist < self.acceptance_radius:
                    self.get_logger().info(f"!!! NOKTA {self.current_waypoint_index+1} TAMAMLANDI !!!")
                    self.current_waypoint_index += 1

        self.counter += 1
        self.command_counter += 1

    # ... Yardımcı Fonksiyonlar Aynı ...
    def send_target(self, lat, lon, alt):
        x, y = self.geo_to_ned(lat, lon, self.home_lat, self.home_lon)
        msg = TrajectorySetpoint()
        msg.position = [x, y, alt]
        msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def geo_to_ned(self, lat, lon, h_lat, h_lon):
        R = 6371000.0
        d_lat = math.radians(lat - h_lat)
        d_lon = math.radians(lon - h_lon)
        lat_rad = math.radians(h_lat)
        return d_lat * R, d_lon * R * math.cos(lat_rad)

    def calculate_dist(self, t_lat, t_lon):
        tx, ty = self.geo_to_ned(t_lat, t_lon, self.home_lat, self.home_lon)
        cx, cy = self.geo_to_ned(self.current_lat, self.current_lon, self.home_lat, self.home_lon)
        return math.sqrt((tx-cx)**2 + (ty-cy)**2)

    def add_meters_to_gps(self, lat, lon, dx, dy):
        R = 6371000.0
        n_lat = lat + (dx/R)*(180/math.pi)
        n_lon = lon + (dy/R)*(180/math.pi)/math.cos(lat*math.pi/180)
        return n_lat, n_lon

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def engage_offboard_mode(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def send_command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()