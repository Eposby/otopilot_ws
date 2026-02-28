#!/usr/bin/env python3
"""
Vision Processor Node — Algı Katmanı (Gözler)
===============================================

Kamera/YOLO tespitlerini veya mock verilerini alır, işler ve
standardize edilmiş TargetInfo mesajı olarak beyne iletir.

İki Mod:
    1. MOCK MOD:  Terminalden TargetInfo yayınlayarak test.
    2. KAMERA MOD: Gerçek kamera görüntüsünde YOLO ile tespit.

Dinlediği Topic'ler:
    /uav1/mock_target       → Mock hedef verisi (geometry_msgs/Point)
                               x, y = local ENU metre, z = irtifa

Yayınladığı Topic:
    /gorev/target_info      → TargetInfo mesajı

Mock Test (terminalden):
    ros2 topic pub --once /uav1/mock_target geometry_msgs/msg/Point "{x: 150.0, y: -40.0, z: 100.0}"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from iha_messages.msg import TargetInfo

# YOLO detector (aynı klasörden import)
try:
    from iha_görev.yolo_detector import YOLODetector
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# cv_bridge: ROS Image ↔ OpenCV dönüşümü
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False


class VisionProcessor(Node):

    def __init__(self):
        super().__init__('vision_processor_node')

        # ═══════════════════════════════════════════════════════════
        # PARAMETRELER
        # ═══════════════════════════════════════════════════════════
        self.camera_topic = '/camera/image_raw'  # İleride değiştirilebilir
        self.frame_width = 640      # Kamera çözünürlüğü (varsayılan)
        self.frame_height = 480
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)
        self.next_target_id = 1     # Otomatik ID sayacı

        # ═══════════════════════════════════════════════════════════
        # MOCK SUBSCRIBER (Test amaçlı — her zaman aktif)
        # ═══════════════════════════════════════════════════════════
        self.mock_target_sub = self.create_subscription(
            TargetInfo, '/uav1/mock_target', self.mock_target_callback, 10)
        # TargetInfo = mesaj tipi iha_messages/TargetInfo buradan geliyor --- TargetInfo: Bu telsizin "dili" gibidir.
        # 10 = mesaj kalitesi (qos) yani mesajın ne kadar kuyrukta bekleyeceği
        # /uav1/mock_target = topic ismi --- Topic: Bu telsizin "kanalı" gibidir. Bu kanal üzerinden mesajlar gönderilir.
        # self.mock_target_callback = callback fonksiyonu: ROS 2 sistemi bu kanal üzerinden bir mesaj kaptığı anda,
        # senin kodundaki mock_target_callback fonksiyonunu otomatik olarak çalıştırır.

        # 2. Bilgi Nereden Alınıyor? Bu bilgiyi kimin gönderdiğinin bir önemi yok.
        # Önemli olan tek şey, birinin /uav1/mock_target kanalına TargetInfo tipinde bir mesaj atmasıdır.
        # Sen (Terminal üzerinden):
        # ros2 topic pub /uav1/mock_target iha_messages/msg/TargetInfo "{detected: true, target_x: 100.0, target_y: 100.0, target_z: 80.0, target_id: 101}"

        # ═══════════════════════════════════════════════════════════
        # KAMERA SUBSCRIBER (YOLO ile gerçek tespit)
        # ═══════════════════════════════════════════════════════════
        self.yolo_detector = None
        self.cv_bridge = None
            # ROS, kamera görüntüsünü kendine has bir "mesaj" formatında gönderir (karmaşık bir sayı dizisi).
            # OpenCV (YOLO) ise görüntüyü bir "matris/numpy array" olarak ister.
            # CvBridge, ROS mesajını alıp YOLO'nun anlayacağı bir fotoğrafa çeviren tercümandır.
        if YOLO_AVAILABLE and CV_BRIDGE_AVAILABLE:
            self.cv_bridge = CvBridge()
            self.yolo_detector = YOLODetector(
                model_path="yolov8n.pt",
                confidence_threshold=0.5,
                target_classes=["airplane"],  # Sabit kanatlı İHA tespiti
                device="auto"
            )
            model_loaded = self.yolo_detector.load_model()

            if model_loaded:
                self.camera_sub = self.create_subscription(
                    Image, self.camera_topic, self.image_callback, 10)
                self.get_logger().info(f"    YOLO aktif! Kamera: {self.camera_topic}")
            else:
                self.get_logger().warn("    YOLO model yüklenemedi — sadece Mock mod aktif")
                self.yolo_detector = None
        else:
            if not YOLO_AVAILABLE:
                self.get_logger().warn("    yolo_detector import edilemedi — sadece Mock mod aktif")
            if not CV_BRIDGE_AVAILABLE:
                self.get_logger().warn("    cv_bridge bulunamadı — sadece Mock mod aktif")

        # ═══════════════════════════════════════════════════════════
        # PUBLISHER
        # ═══════════════════════════════════════════════════════════
        self.target_pub = self.create_publisher(TargetInfo, '/gorev/target_info', 10)

        # ═══════════════════════════════════════════════════════════
        # İÇ DURUM
        # ═══════════════════════════════════════════════════════════
        self.last_detected = False
        self.target_timeout = 3.0  # saniye — hedef bu süre gelmezse "kayıp"
        self.last_detection_time = 0.0

        # ═══ Timer: Hedef kayıp kontrolü (5 Hz) ═══
        self.timer = self.create_timer(0.2, self.check_target_timeout)

        self.get_logger().info("═══ Vision Processor başlatıldı ═══")
        self.get_logger().info(f"    Mock topic: /uav1/mock_target (TargetInfo)")
        self.get_logger().info(f"    YOLO modu: {'AKTİF' if self.yolo_detector else 'KAPALI'}")

    # ═══════════════════════════════════════════════════════════════
    # MOCK HEDEF CALLBACK (Test için — terminalden veri gönderilir)
    # ═══════════════════════════════════════════════════════════════
    def mock_target_callback(self, msg):
        """Mock kanaldan gelen veriyi (X, Y, Z, ID vb.) doğrudan yayınla."""
        # Son görülme zamanını güncelle
        self.last_detection_time = self.get_clock().now().nanoseconds / 1e9

        # Gelen mesajı doğrudan (veya ufak kontrollerle) tekrar yayınla
        target_msg = msg
        target_msg.detected = True

        # ID yoksa (0 gelmişse) varsayılan ata
        if target_msg.target_id == 0:
            target_msg.target_id = 999

        self.target_pub.publish(target_msg)

        if not self.last_detected:
            self.get_logger().info(f"Hedef Tespit Edildi! ID: {target_msg.target_id}")
            self.last_detected = True

        if self.get_clock().now().nanoseconds % 10 == 0:
            self.get_logger().info(
                f"  MOCK UPD → ID:{target_msg.target_id} ({target_msg.target_x:.1f}, {target_msg.target_y:.1f}, {target_msg.target_z:.1f}m)")

    # ═══════════════════════════════════════════════════════════════
    # KAMERA CALLBACK (Gerçek YOLO tespiti)
    # ═══════════════════════════════════════════════════════════════
    def image_callback(self, msg):
        """
        Kameradan gelen her frame'i YOLO'ya ver, tespit varsa TargetInfo yayınla.
        Piksel hatasını (error_x, error_y) hesaplar — ileride PID'ye girdi olacak.
        """
        if self.yolo_detector is None or self.cv_bridge is None:
            return

        # 1. ROS Image → OpenCV frame dönüşümü
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Frame dönüşüm hatası: {e}")
            return

        # Frame boyutlarını güncelle (ilk frame'de doğru değeri yakala)
        h, w = frame.shape[:2]
        if w != self.frame_width or h != self.frame_height:
            self.frame_width = w
            self.frame_height = h
            self.frame_center = (w // 2, h // 2)

        # 2. YOLO tespiti
        detections = self.yolo_detector.detect(frame)

        # 3. Birincil hedefi seç
        primary = self.yolo_detector.get_primary_target(detections)

        if primary is not None:
            # 4. Piksel hatasını hesapla (PID'ye hazır)
            error_x, error_y = self.yolo_detector.calculate_error(
                primary, self.frame_center)

            # 5. TargetInfo mesajı oluştur
            target_msg = TargetInfo()
            target_msg.detected = True
            target_msg.target_x = float(primary.center[0])    # Piksel X
            target_msg.target_y = float(primary.center[1])    # Piksel Y
            target_msg.target_z = 0.0                         # Derinlik: ileride eklenecek
            target_msg.confidence = primary.confidence
            target_msg.bbox_cx = float(primary.center[0])
            target_msg.bbox_cy = float(primary.center[1])
            target_msg.bbox_w = float(primary.bbox[2] - primary.bbox[0])
            target_msg.bbox_h = float(primary.bbox[3] - primary.bbox[1])
            target_msg.target_class = primary.class_name
            target_msg.target_id = self.next_target_id        # Sabit ID (tek hedef)
            target_msg.error_x = float(error_x)               # PID girdi X
            target_msg.error_y = float(error_y)               # PID girdi Y

            self.target_pub.publish(target_msg)

            # Durum güncelle
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
            if not self.last_detected:
                self.get_logger().info(
                    f"YOLO Hedef Tespit! Sınıf: {primary.class_name}, "
                    f"Güven: {primary.confidence:.2f}")
                self.last_detected = True

    # ═══════════════════════════════════════════════════════════════
    # HEDEF KAYIP KONTROLÜ
    # ═══════════════════════════════════════════════════════════════
    def check_target_timeout(self):
        """Hedef belirli süre gelmezse 'kayıp' mesajı yayınla."""
        if not self.last_detected:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.last_detection_time

        if elapsed > self.target_timeout:
            # Hedef kayboldu
            lost_msg = TargetInfo()
            lost_msg.detected = False
            lost_msg.target_class = "lost"
            self.target_pub.publish(lost_msg)
            self.last_detected = False
            self.get_logger().warn("HEDEF KAYBEDILDI (timeout)")


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
