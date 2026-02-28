#!/usr/bin/env python3
"""
QR Mission Modülü - State Machine'den çağrılır
"""
from enum import Enum
import math


class MissionStatus(Enum):
    """Görev durumu."""
    RUNNING = 0
    COMPLETED = 1
    FAILED = 2


class QRMission:
    """
    QR görevi için sınıf.
    main.py'deki state machine bu sınıfı kullanarak görevi yönetir.
    """
    
    def __init__(self, node):
        """
        Args:
            node: Ana ROS2 node referansı (publisher, subscriber erişimi için)
        """
        self.node = node
        self.task_state = 0
        self.status = MissionStatus.RUNNING
        
        # Görev parametreleri
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.pull_up_alt = 60.0
        
        # İç değişkenler
        self.kamikaze_yaw = 0.0
        self.kamikaze_angle = 0.0
        self.kamikaze_start_alt = 0.0
        self.inc_target_x = 0.0
        self.inc_target_y = 0.0
        self.inc_target_z = 0.0
        self.log_counter = 0
        self.offboard_counter = 0
        self.pullup_counter = 0
    
    def configure(self, target_lat: float, target_lon: float, pull_up_alt: float = 60.0):
        """Görevi yapılandır."""
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.pull_up_alt = pull_up_alt
        self.task_state = 0
        self.status = MissionStatus.RUNNING
    
    def execute(self) -> MissionStatus:
        """
        Her döngüde çağrılır. Görev durumunu döndürür.
        
        Returns:
            MissionStatus: RUNNING, COMPLETED veya FAILED
        """
        if self.status != MissionStatus.RUNNING:
            return self.status
        
        # Hedef koordinatları local'e çevir
        tx, ty = self.node.global_to_local(self.target_lat, self.target_lon)
        
        # === TASK STATE 0: Hazırlık ===
        if self.task_state == 0:
            self._init_kamikaze(tx, ty)
        
        # === TASK STATE 1: OFFBOARD Bekle ===
        elif self.task_state == 1:
            self._wait_offboard()
        
        # === TASK STATE 2: Yaklaşma ===
        elif self.task_state == 2:
            self._approach(tx, ty)
        
        # === TASK STATE 3: Dalış ===
        elif self.task_state == 3:
            self._dive(tx, ty)
        
        # === TASK STATE 4: Pull-up ===
        elif self.task_state == 4:
            self._pullup()
        
        # === TASK STATE 5: Tamamlandı ===
        elif self.task_state == 5:
            self.status = MissionStatus.COMPLETED
        
        return self.status
    
    # ==================== GÖREV AŞAMALARI ====================
    def _init_kamikaze(self, tx, ty):
        """Başlangıç değerlerini ayarla."""
        self.node.get_logger().info(">> QR MISSION: Kamikaze Başladı")
        
        pos = self.node.current_pos
        self.kamikaze_start_alt = pos['z']
        
        dx = tx - pos['x']
        dy = ty - pos['y']
        self.kamikaze_yaw = math.degrees(math.atan2(dy, dx))
        self.kamikaze_angle = math.atan2(dy, dx)
        
        self.inc_target_x = pos['x']
        self.inc_target_y = pos['y']
        self.inc_target_z = pos['z']
        
        # Mevcut konumu setpoint olarak gönder
        self.node.publish_setpoint(pos['x'], pos['y'], pos['z'])
        self.node.set_mode_command("OFFBOARD")
        
        self.task_state = 1
        self.offboard_counter = 0
    
    def _wait_offboard(self):
        """OFFBOARD modunu bekle."""
        pos = self.node.current_pos
        self.node.publish_setpoint(pos['x'], pos['y'], pos['z'])
        self.offboard_counter += 1
        
        if self.offboard_counter > 20:
            self.node.get_logger().info(">> OFFBOARD aktif")
            self.inc_target_x = pos['x']
            self.inc_target_y = pos['y']
            self.inc_target_z = pos['z']
            self.task_state = 2
    
    def _approach(self, tx, ty):
        """Hedefe yaklaş."""
        pos = self.node.current_pos
        step_size = 2.0
        
        dx = tx - pos['x']
        dy = ty - pos['y']
        dist = math.sqrt(dx**2 + dy**2)
        
        self.inc_target_x += step_size * math.cos(self.kamikaze_angle)
        self.inc_target_y += step_size * math.sin(self.kamikaze_angle)
        self.inc_target_z = self.kamikaze_start_alt
        
        self.node.publish_setpoint(self.inc_target_x, self.inc_target_y, self.inc_target_z)
        
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.node.get_logger().info(f"   Yaklaşma | Mesafe: {dist:.1f}m")
        
        if dist < 100.0:
            self.node.get_logger().info(">> 100m - Dalışa Geçiliyor")
            self.task_state = 3
    
    def _dive(self, tx, ty):
        """45 derece dalış."""
        pos = self.node.current_pos
        
        dx = tx - pos['x']
        dy = ty - pos['y']
        dist = math.sqrt(dx**2 + dy**2)
        
        self.node.publish_attitude(
            roll=0.0,
            pitch=-45.0,
            yaw=self.kamikaze_yaw,
            thrust=0.0
        )
        
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.node.get_logger().info(f"   DALIŞ | Mesafe: {dist:.1f}m | İrtifa: {pos['z']:.1f}m")
        
        if pos['z'] <= (self.pull_up_alt + 5.0):
            self.node.get_logger().info(">> Pull-up Başlıyor")
            self.task_state = 4
            self.pullup_counter = 0
    
    def _pullup(self):
        """Pull-up manevra."""
        pos = self.node.current_pos
        
        self.node.publish_attitude(
            roll=0.0,
            pitch=30.0,
            yaw=self.kamikaze_yaw,
            thrust=1.0
        )
        
        self.pullup_counter += 1
        if self.pullup_counter % 20 == 0:
            self.node.get_logger().info(f"   PULL-UP | İrtifa: {pos['z']:.1f}m")
        
        if pos['z'] > 50.0:
            self.node.get_logger().info(">> QR Mission Tamamlandı")
            self.task_state = 5