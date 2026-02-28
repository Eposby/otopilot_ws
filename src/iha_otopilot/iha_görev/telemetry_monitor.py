#!/usr/bin/env python3
"""
Telemetry Monitor Node — Algı Katmanı
======================================
Bu yaptığın işleme yazılım mimarisinde "Data Aggregation" (Veri Birleştirme/Toplama) denir.



MAVROS'tan gelen ham telemetri verilerini toplar, süzer ve temiz bir
TelemetryStatus raporu olarak yayınlar.

Dinlediği Topic'ler:
    /mavros/state                      → Bağlantı durumu, mod
    /mavros/global_position/global     → GPS koordinatları
    /mavros/local_position/pose        → Local ENU pozisyon
    /mavros/battery                    → Batarya durumu
    /mavros/vfr_hud                    → Airspeed, groundspeed

Yayınladığı Topic:
    /gorev/telemetry_status            → TelemetryStatus mesajı


TelemetryStatus.msg:

bool detected
float32 target_x
float32 target_y
float32 target_z
float32 confidence
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
string target_class
int32 target_id 
bool is_connected bu ileride kullanılmak üzere eklendi loglama için kullanılacak



Topic: /mavros/global_position/global
Mesaj Tipi: sensor_msgs/msg/NavSatFix

# Örnek Veri:
latitude: 39.9255       # Enlem (Kuzey-Güney çizgisi)
longitude: 32.8662      # Boylam (Doğu-Batı çizgisi)
altitude: 950.4         # Deniz seviyesinden yükseklik (MSL - Mean Sea Level)
status.status: 0        # 0 = GPS Fix (Uyduya kilitlendi)
status.service: 1       # GPS servisi aktif



"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import State, VFR_HUD
from iha_messages.msg import TelemetryStatus


# ═══════════════════════════════════════════════════════════════
# GÜVENLİK EŞİKLERİ
# ═══════════════════════════════════════════════════════════════
STALL_SPEED = 12.0          # Stall hızı (m/s) — bunun altı kritik
BATTERY_CRITICAL = 0.15     # Batarya kritik eşiği (%15)
GPS_MIN_SATELLITES = 6      # Minimum uydu sayısı


class TelemetryMonitor(Node):

    def __init__(self):
        super().__init__('telemetry_monitor_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ═══ Subscribers ═══
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        # uçağın o anki durumu (uçuş modu, bağlantı durumu vb.) connected, armed, mode, system_status
        # Örnek Veri:
        # connected: True          # Haberleşme var
        # armed: True              # Motorlar dönüyor
        # mode: "OFFBOARD"         # Kontrol bizim kodumuzda
        # system_status: 4         # 4 = Active (Uçuşa hazır/Uçuyor)

        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos_profile)
        # uçağın o anki konumu (x, y, z)
        # Örnek Veri:
        # pose.position.x: 125.40   # Kalkıştan 125 metre Doğu'da
        # pose.position.y: -42.15   # Kalkıştan 42 metre Güney'de
        # pose.position.z: 100.0    # Yerden tam 100 metre yüksekte
        # pose.orientation: ...     # Uçağın yatış (roll/pitch/yaw) açısı (kuaterniyon olarak)


        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile)
        # uçağın o anki batarya durumu
        # Örnek Veri:
        # voltage: 15.4             # 4S batarya için yaklaşık voltaj
        # percentage: 0.82          # %82 dolu (Biz bunu kodda %100 ile çarpıp gösteriyoruz)
        # current: 12.5             # Anlık çekilen akım (Amper)


        self.vfr_sub = self.create_subscription(VFR_HUD, '/mavros/vfr_hud', self.vfr_callback, qos_profile)
        # uçağın o anki hızı, irtifası vb.
        # Örnek Veri:
        # airspeed: 18.5            # Havada tutunma hızı (m/s) - Rüzgara karşı ölçülen hız
        # groundspeed: 20.2         # Yere göre hızı (m/s)
        # altitude: 100.0           # Deniz seviyesine veya kalkışa göre irtifa
        # heading: 90               # Pusula yönü (90 = Doğu'ya bakıyor)
        # throttle: 0.65            # Motorların %65 güçle çalıştığını söyler (0 ile 1 arası)

        # ═══ Publisher ═══
        self.telemetry_pub = self.create_publisher(TelemetryStatus, '/gorev/telemetry_status', 10)

        # ═══ İç Durum ═══
        self.mavros_connected = False
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.battery_voltage = 0.0
        self.battery_remaining = 1.0
        self.gps_fix_type = 0
        self.gps_satellites = 0

        # ═══ Timer: 10 Hz Rapor ═══
        self.timer = self.create_timer(0.1, self.publish_telemetry)
        self.log_counter = 0

        self.get_logger().info("═══ Telemetry Monitor başlatıldı ═══")

    # ═══════════════════════════════════════════════════════════════
    # RAPOR YAYINI
    # ═══════════════════════════════════════════════════════════════
    def publish_telemetry(self):
        """10 Hz'de telemetri raporu yayınla."""
        msg = TelemetryStatus()
        msg.airspeed = self.airspeed
        msg.groundspeed = self.groundspeed
        msg.altitude = self.pos_z
        msg.battery_voltage = self.battery_voltage
        msg.battery_remaining = self.battery_remaining
        msg.gps_fix_type = self.gps_fix_type
        msg.gps_satellites = self.gps_satellites
        msg.pos_x = self.pos_x
        msg.pos_y = self.pos_y
        msg.pos_z = self.pos_z
        msg.is_connected = self.mavros_connected

        # ─── Kritik Durum Değerlendirmesi ───
        warnings = []
        if self.airspeed > 0 and self.airspeed < STALL_SPEED:
            warnings.append(f"STALL_RISK: Hız {self.airspeed:.1f} m/s")
        if self.battery_remaining < BATTERY_CRITICAL:
            warnings.append(f"BATARYA_KRITIK: %{self.battery_remaining*100:.0f}")
        if self.gps_satellites < GPS_MIN_SATELLITES:
            warnings.append(f"GPS_ZAYIF: {self.gps_satellites} uydu")

        msg.is_critical = len(warnings) > 0
        msg.warning_message = " | ".join(warnings) if warnings else ""
        # Mantığı: Eğer birden fazla uyarı varsa (örneğin hem batarya düşük hem GPS zayıf), bu uyarıları aralarına " | " işareti koyarak birleştirir ve tek bir uzun yazı haline getirir.
        # Örnek: Eğer warnings listesi ["BATARYA_KRITIK: %10", "GPS_ZAYIF: 4 uydu"] ise, 
        # msg.warning_message "BATARYA_KRITIK: %10 | GPS_ZAYIF: 4 uydu" olur.


        self.telemetry_pub.publish(msg)

        # ─── Periyodik Loglama ───
        #1. if self.log_counter % 50 == 0: Bu bir **"Hız Düzenleyici"**dir.Bu fonksiyon (publish_telemetry) saniyede 10 kez çalışıyor.% 50 (mod 50) demek; her 50 çalışmada bir (yani her 5 saniyede bir) ekrana yazı bas demektir.Amacı: Terminalin çok hızlı akıp gitmesini ve bilgisayarı yormasını engellem
        if self.log_counter % 50 == 0:
            status = "🔴 KRİTİK" if msg.is_critical else "🟢 NORMAL"
            self.get_logger().info(
                f"   TEL | {status} | Hız: {self.airspeed:.1f} m/s "
                f"| Alt: {self.pos_z:.0f}m | Bat: %{self.battery_remaining*100:.0f} "
                f"| GPS: {self.gps_satellites} uydu")
            if msg.is_critical:
                self.get_logger().warn(f"   ⚠ {msg.warning_message}")
        self.log_counter += 1

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════
    def state_callback(self, msg):
        self.mavros_connected = msg.connected

    def local_pos_callback(self, msg):
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        self.pos_z = msg.pose.position.z

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_remaining = msg.percentage if msg.percentage >= 0 else 1.0

    def vfr_callback(self, msg):
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
