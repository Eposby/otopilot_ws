#!/usr/bin/env python3
"""
Safety Watchdog Node — Refleks Katmanı
=======================================

State Machine'in üstünde, sürekli tetikte bekleyen "omurilik refleksi".
Telemetri verilerini analiz eder ve hayati tehlike durumunda beynin
kararını ezerek doğrudan güvenlik komutunu yayınlar.

Dinlediği Topic:
    /gorev/telemetry_status    → TelemetryStatus

Yayınladığı Topic:
    /gorev/safety_override     → SafetyOverride

Acil Durum Protokolleri:
    - STALL_RISK   → override_mode: FULL_THROTTLE
    - BATARYA_KRITIK → override_mode: RTL
    - GPS_KAYIP    → override_mode: LOITER
"""

import rclpy
from rclpy.node import Node
from iha_messages.msg import TelemetryStatus, SafetyOverride


# ═══════════════════════════════════════════════════════════════
# GÜVENLİK EŞİKLERİ
# ═══════════════════════════════════════════════════════════════
STALL_SPEED = 12.0          # Stall hızı eşiği (m/s)
BATTERY_EMERGENCY = 0.10    # Batarya acil durum (%10)
GPS_MIN_FIX = 3             # Minimum GPS fix type (3D fix)
MIN_ALTITUDE = 50.0         # Minimum güvenli irtifa (m)


class SafetyWatchdog(Node):

    def __init__(self):
        super().__init__('safety_watchdog_node')

        # ═══ Subscribers ═══
        self.telemetry_sub = self.create_subscription(
            TelemetryStatus, '/gorev/telemetry_status',
            self.telemetry_callback, 10)

        # ═══ Publisher ═══
        self.override_pub = self.create_publisher(
            SafetyOverride, '/gorev/safety_override', 10)

        # ═══ İç Durum ═══
        self.last_telemetry = None
        self.override_active = False
        self.consecutive_critical = 0  # Art arda kritik sayacı (false positive filtre)
        self.CRITICAL_THRESHOLD = 10   # 10 art arda kritik = gerçek tehlike (1 saniye)

        # ═══ Timer: 20 Hz Güvenlik Denetimi ═══
        self.timer = self.create_timer(0.05, self.safety_check)
        self.log_counter = 0

        self.get_logger().info("═══ Safety Watchdog başlatıldı ═══")
        self.get_logger().info(f"    Stall eşiği: {STALL_SPEED} m/s")
        self.get_logger().info(f"    Batarya acil: %{BATTERY_EMERGENCY*100:.0f}")

    # ═══════════════════════════════════════════════════════════════
    # GÜVENLİK DENETİMİ
    # ═══════════════════════════════════════════════════════════════
    def safety_check(self):
        """20 Hz'de güvenlik analizi yap."""
        if self.last_telemetry is None:
            return

        tel = self.last_telemetry
        override_msg = SafetyOverride()
        override_msg.override_active = False
        override_msg.override_mode = ""
        override_msg.reason = ""
        override_msg.override_alt = 0.0

        # ─── Tehlike Değerlendirmesi (Öncelik Sırasına Göre) ───

        # 1. STALL RİSKİ — En yüksek öncelik
        if tel.airspeed > 0 and tel.airspeed < STALL_SPEED and tel.altitude > 10.0:
            override_msg.override_active = True
            override_msg.override_mode = "FULL_THROTTLE"
            override_msg.reason = f"STALL RISKI! Hiz: {tel.airspeed:.1f} m/s"

        # 2. BATARYA ACİL DURUM
        elif tel.battery_remaining > 0 and tel.battery_remaining < BATTERY_EMERGENCY:
            override_msg.override_active = True
            override_msg.override_mode = "RTL"
            override_msg.reason = f"BATARYA ACIL! %{tel.battery_remaining*100:.0f}"

        # 3. GPS KAYBI
        elif tel.gps_satellites < 4:
            override_msg.override_active = True
            override_msg.override_mode = "LOITER"
            override_msg.reason = f"GPS KAYBI! {tel.gps_satellites} uydu"

        # 4. İRTİFA TEHLİKESİ
        elif 0 < tel.altitude < MIN_ALTITUDE and tel.airspeed > 5.0:
            override_msg.override_active = True
            override_msg.override_mode = "CLIMB"
            override_msg.reason = f"IRTIFA KRITIK! {tel.altitude:.0f}m"
            override_msg.override_alt = MIN_ALTITUDE + 20.0

        # ─── Art Arda Kritik Filtresi (False Positive Engelleme) ───
        if override_msg.override_active:
            self.consecutive_critical += 1
        else:
            self.consecutive_critical = 0

        # Sadece art arda N kez kritik gelirse override yayınla
        if self.consecutive_critical >= self.CRITICAL_THRESHOLD:
            if not self.override_active:
                self.get_logger().error(
                    f"  🚨 ACİL OVERRIDE: {override_msg.override_mode} "
                    f"— {override_msg.reason}")
            self.override_active = True
            self.override_pub.publish(override_msg)
        else:
            # Tehlike geçtiyse override'ı kaldır
            if self.override_active:
                self.get_logger().info("  ✓ Tehlike geçti — Override kaldırıldı")
                cancel_msg = SafetyOverride()
                cancel_msg.override_active = False
                cancel_msg.override_mode = ""
                cancel_msg.reason = "Tehlike gecti"
                self.override_pub.publish(cancel_msg)
                self.override_active = False
                self.consecutive_critical = 0

        # ─── Periyodik Loglama ───
        if self.log_counter % 100 == 0:  # Her 5 saniyede bir
            status = "🛡 OVERRIDE AKTİF" if self.override_active else "✓ NORMAL"
            self.get_logger().info(f"   WATCHDOG | {status}")
        self.log_counter += 1

    # ═══════════════════════════════════════════════════════════════
    # CALLBACK
    # ═══════════════════════════════════════════════════════════════
    def telemetry_callback(self, msg):
        self.last_telemetry = msg


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
