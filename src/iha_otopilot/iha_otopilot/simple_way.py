#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAirData,
    VehicleGlobalPosition
)

# ================== SABİTLER ==================
V_TAKEOFF = 18.0
V_CRUISE  = 22.0
V_LAND    = 15.0

CLIMB_RATE   = -3.0   # NED (yukarı negatif)
DESCENT_RATE =  2.0

WAYPOINT_RADIUS = 20.0  # metre
# ==============================================


class FixedWingAutopilot(Node):

    def __init__(self):
        super().__init__('fw_autopilot')

        # -------- Publishers --------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # -------- Subscribers (QoS ÖNEMLİ) --------
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            VehicleAirData,
            '/fmu/out/vehicle_air_data',
            self.air_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_cb,
            qos_profile_sensor_data)

        # 20 Hz (OFFBOARD FAILSAFE İÇİN ZORUNLU)
        self.timer = self.create_timer(0.05, self.loop)

        # -------- State --------
        self.state = "IDLE"
        self.armed = False
        self.nav_state = None
        self.airspeed = 0.0
        self.gps = None
        self.home = None

        # Zurich civarı örnek waypoint’ler
        self.waypoints = [
            (47.3770, 8.5415, 480),
            (47.3780, 8.5450, 480),
            (47.3755, 8.5480, 480)
        ]
        self.wp_index = 0

    # ================= CALLBACKS =================

    def local_pos_cb(self, msg):
        self.local_pos = msg

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def air_cb(self, msg):
        self.airspeed = msg.true_airspeed_m_s

    def gps_cb(self, msg):
        self.gps = msg
        if self.home is None:
            self.home = (msg.lat, msg.lon, msg.alt)

    # ================= HELPERS =================

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard(self):
        self.send_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0, 6.0
        )

    def set_throttle(self, value):
        # PX4 fixed-wing throttle
        self.send_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR,
            3.0, value
        )

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.velocity = True
        msg.attitude = True
        self.offboard_pub.publish(msg)

    # ================= GUIDANCE =================

    def heading_to(self, lat, lon):
        dlat = math.radians(lat - self.gps.lat)
        dlon = math.radians(lon - self.gps.lon)
        return math.atan2(dlon, dlat)

    def reached_wp(self, lat, lon):
        dlat = (lat - self.gps.lat) * 111000
        dlon = (lon - self.gps.lon) * 111000
        return math.hypot(dlat, dlon) < WAYPOINT_RADIUS

    # ================= MAIN LOOP =================

    def loop(self):

        # OFFBOARD heartbeat HER ZAMAN
        self.publish_offboard_heartbeat()

        if self.gps is None:
            return

        # -------- IDLE --------
        if self.state == "IDLE":
            self.arm()
            self.state = "TAKEOFF"
            self.get_logger().info("→ TAKEOFF")

        # -------- TAKEOFF --------
        elif self.state == "TAKEOFF":

            # %100 gaz, pistten hızlanma
            self.set_throttle(1.0)

            sp = TrajectorySetpoint()
            sp.velocity[0] = V_TAKEOFF
            sp.velocity[2] = CLIMB_RATE
            sp.yaw = self.heading_to(*self.waypoints[0][:2])
            self.traj_pub.publish(sp)

            # AIRSPEED oluşmadan OFFBOARD YOK
            if self.airspeed > V_TAKEOFF:
                self.set_offboard()
                self.state = "MISSION"
                self.get_logger().info("→ OFFBOARD + MISSION")

        # -------- MISSION --------
        elif self.state == "MISSION":

            lat, lon, alt = self.waypoints[self.wp_index]

            sp = TrajectorySetpoint()
            sp.velocity[0] = V_CRUISE
            sp.yaw = self.heading_to(lat, lon)
            self.traj_pub.publish(sp)

            if self.reached_wp(lat, lon):
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints):
                    self.state = "RETURN"
                    self.get_logger().info("→ RETURN")

        # -------- RETURN --------
        elif self.state == "RETURN":

            lat, lon, alt = self.home

            sp = TrajectorySetpoint()
            sp.velocity[0] = V_CRUISE
            sp.yaw = self.heading_to(lat, lon)
            self.traj_pub.publish(sp)

            if self.reached_wp(lat, lon):
                self.state = "LAND"
                self.get_logger().info("→ LAND")

        # -------- LAND --------
        elif self.state == "LAND":

            self.set_throttle(0.3)

            sp = TrajectorySetpoint()
            sp.velocity[0] = V_LAND
            sp.velocity[2] = DESCENT_RATE
            sp.yaw = self.heading_to(*self.home[:2])
            self.traj_pub.publish(sp)


# ================= MAIN =================

def main():
    rclpy.init()
    node = FixedWingAutopilot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
