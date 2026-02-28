#!/usr/bin/env python3
"""
Trajectory Generator Node — Yörünge Katmanı
=============================================

Beynin taktiksel emirlerini alır ve uçağın fiziksel olarak
uygulayabileceği setpoint'lere dönüştürür.

Safety Watchdog'dan override gelirse beynin kararını ezer.

Dinlediği Topic'ler:
    /gorev/mission_command     → MissionCommand (Beyin)
    /gorev/safety_override     → SafetyOverride (Watchdog)

Yayınladığı Topic'ler:
    /mavros/setpoint_position/local → PoseStamped setpoint

MAVROS Service'leri:
    /mavros/set_mode           → Override durumunda mod değişikliği
    /mavros/param/set          → Hız parametreleri
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
from iha_messages.msg import MissionCommand, SafetyOverride


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator_node')

        # ═══ Subscribers ═══
        self.command_sub = self.create_subscription(
            MissionCommand, '/gorev/mission_command',
            self.command_callback, 10)
        self.override_sub = self.create_subscription(
            SafetyOverride, '/gorev/safety_override',
            self.override_callback, 10)

        # ═══ Publishers ═══
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # ═══ Service Clients ═══
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(
            ParamSetV2, '/mavros/param/set')

        # ═══ İç Durum ═══
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 100.0
        self.target_speed = 18.0
        self.current_set_speed = 0.0
        self.enable_setpoint = False
        self.last_command_type = ""

        # Safety Override
        self.override_active = False
        self.override_mode = ""

        # ═══ Timer: 10 Hz Sürekli Setpoint ═══
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint_loop)
        self.log_counter = 0

        self.get_logger().info("═══ Trajectory Generator başlatıldı ═══")

    # ═══════════════════════════════════════════════════════════════
    # SÜREKLİ SETPOINT YAYINI (10 Hz)
    # ═══════════════════════════════════════════════════════════════
    def publish_setpoint_loop(self):
        """10 Hz'de MAVROS'a setpoint gönder — OFFBOARD stabilite."""
        if not self.enable_setpoint:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

        if self.log_counter % 50 == 0:
            src = "🛡 OVERRIDE" if self.override_active else f"📍 {self.last_command_type}"
            self.get_logger().info(
                f"   TRAJ | {src} | → ({self.target_x:.0f}, {self.target_y:.0f}, "
                f"{self.target_z:.0f}m) | Hız: {self.target_speed:.0f} m/s")
        self.log_counter += 1

    # ═══════════════════════════════════════════════════════════════
    # BEYİN EMRİ CALLBACK
    # ═══════════════════════════════════════════════════════════════
    def command_callback(self, msg):
        """
        Beynin taktiksel emrini al.
        Override aktifse beynin kararını yoksay.
        """
        if self.override_active:
            return  # Watchdog kontrol ediyor — beyni dinleme

        self.target_x = msg.target_x
        self.target_y = msg.target_y
        self.target_z = msg.target_z
        self.target_speed = msg.speed
        self.last_command_type = msg.command_type
        self.enable_setpoint = True

        # Hız değişikliği varsa PX4 parametrelerini güncelle
        self.update_speed(msg.speed)

    # ═══════════════════════════════════════════════════════════════
    # SAFETY OVERRIDE CALLBACK
    # ═══════════════════════════════════════════════════════════════
    def override_callback(self, msg):
        """
        Watchdog'dan acil durum override'ı.
        Aktif → beynin kararını ez, doğrudan güvenlik komutunu uygula.
        İnaktif → normale dön.
        """
        if msg.override_active and not self.override_active:
            # Override BAŞLADI
            self.override_active = True
            self.override_mode = msg.override_mode
            self.get_logger().error(
                f"  🚨 OVERRIDE AKTİF: {msg.override_mode} — {msg.reason}")

            # Mod bazlı acil eylem
            if msg.override_mode == "LOITER":
                self.set_mode("AUTO.LOITER")
                self.enable_setpoint = False
            elif msg.override_mode == "RTL":
                self.set_mode("AUTO.RTL")
                self.enable_setpoint = False
            elif msg.override_mode == "FULL_THROTTLE":
                # Stall kurtarma: hızı maksimuma çek
                self.update_speed(25.0)
            elif msg.override_mode == "CLIMB":
                # İrtifa kurtarma: hedef irtifayı override et
                if msg.override_alt > 0:
                    self.target_z = msg.override_alt
                    self.get_logger().warn(
                        f"  ⬆ İrtifa override: {msg.override_alt:.0f}m")

        elif not msg.override_active and self.override_active:
            # Override SONA ERDİ
            self.override_active = False
            self.override_mode = ""
            self.get_logger().info("  ✓ Override kaldırıldı — Beyin kontrolü geri alıyor")

    # ═══════════════════════════════════════════════════════════════
    # HIZ KONTROL
    # ═══════════════════════════════════════════════════════════════
    def update_speed(self, target_speed):
        """PX4 seyir hızı parametrelerini dinamik güncelle."""
        if self.current_set_speed == target_speed:
            return

        is_slowing = target_speed < self.current_set_speed
        self.current_set_speed = target_speed
        trim = target_speed - 1.0

        if is_slowing or target_speed == 0.0:
            self._set_param("FW_AIRSPD_TRIM", trim)
            self._set_param("FW_AIRSPD_MAX", target_speed)
        else:
            self._set_param("FW_AIRSPD_MAX", target_speed)
            self._set_param("FW_AIRSPD_TRIM", trim)

    def _set_param(self, param_id, value):
        req = ParamSetV2.Request()
        req.param_id = param_id
        req.value = ParameterValue()
        req.value.type = ParameterType.PARAMETER_DOUBLE
        req.value.double_value = float(value)
        self.param_set_client.call_async(req)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
