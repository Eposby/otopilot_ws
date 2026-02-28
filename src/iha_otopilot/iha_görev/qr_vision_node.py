#!/usr/bin/env python3
"""
QR Vision Node — Kamikaze QR Hedef Takip Sistemi (ROS 2)
=========================================================

Gazebo kamerasından gelen görüntüyü işleyerek:
  1. YOLOv8 ile hedef arar (SEARCH modu)
  2. Kalman Filtresi ile hedefi takip eder (TRACK modu)
  3. WeChat QR Code ile hedef üzerindeki QR'ı okur
  4. Sonuçları ROS 2 topic'lerine yayınlar

Yayınladığı Topic'ler:
  /gorev/qr_target_info   → TargetInfo (hedef konum, piksel hatası, bbox, QR durumu)
  /gorev/qr_result         → std_msgs/String (okunan QR metni)

Dinlediği Topic:
  /camera/image_raw        → sensor_msgs/Image (Gazebo kamerası)

Çalıştırma:
  ros2 run iha_otopilot qr_vision
"""

import os
import time
import multiprocessing

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# cv_bridge yerine manuel dönüşüm (NumPy versiyon uyumsuzluğunu önler)
import numpy as np

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# Custom mesaj
from iha_messages.msg import TargetInfo

# Aynı paketteki destek modülleri
from iha_görev.qr_tracker import KalmanBoxTracker
from iha_görev.qr_reader import AsyncQRReader

# Model dizini
_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')


# ═══════════════════════════════════════════════════════════════
# YAPILANDIRMA PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
SEARCH_SIZE = 640           # SEARCH modunda resize boyutu
TRACK_ROI_SIZE = 640        # TRACK modunda ROI boyutu
CONF_THRESHOLD = 0.4        # YOLO güven eşiği
LOST_THRESHOLD = 30         # Kaç frame kaybedilirse SEARCH'e dön
QR_PADDING = 0.2            # QR crop çerçevesine padding oranı


class QRVisionNode(Node):

    def __init__(self):
        super().__init__('qr_vision_node')

        # ═══ Durum Kontrolleri ═══
        if not YOLO_AVAILABLE:
            self.get_logger().fatal("ultralytics bulunamadı! 'pip install ultralytics' gerekli.")
            return

        # ═══ YOLO Model Yükleme ═══
        model_path = os.path.join(_MODELS_DIR, 'best.pt')
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ YOLO model yüklendi: {model_path}")
        except Exception as e:
            self.get_logger().fatal(f"❌ YOLO model yüklenemedi: {e}")
            return

        # ═══ QR Okuyucu (Multiprocessing) ═══
        self.qr_result_queue = multiprocessing.Queue()
        self.qr_control_queue = multiprocessing.Queue()
        self.qr_reader = AsyncQRReader(self.qr_result_queue, self.qr_control_queue)
        self.qr_reader.start()

        # ═══ Kalman Tracker ═══
        self.tracker = KalmanBoxTracker()

        # ═══ Durum Değişkenleri ═══
        self.state = "SEARCH"       # SEARCH veya TRACK
        self.missed_count = 0
        self.locked_qr = None       # Okunan QR metni (kilitlendi mi?)
        self.last_known_box = None   # Son bilinen hedef kutusu [x, y, w, h]
        self.prev_time = time.time()

        # Frame boyutları (ilk frame'de güncellenecek)
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center = (320, 240)

        # ═══ ROS 2 Parametreleri ═══
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        # ═══ ROS 2 Subscriber (BEST_EFFORT QoS — Gazebo kamera ile uyumlu) ═══
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, sensor_qos)

        # ═══ ROS 2 Publishers ═══
        self.target_pub = self.create_publisher(TargetInfo, '/gorev/qr_target_info', 10)
        self.qr_text_pub = self.create_publisher(String, '/gorev/qr_result', 10)

        # ═══ GUI Penceresi ═══
        self.win_name = "KAMIKAZE QR SYSTEM"
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("═══ QR Vision Node başlatıldı ═══")
        self.get_logger().info(f"    Kamera: {camera_topic}")
        self.get_logger().info(f"    QR Target: /gorev/qr_target_info")
        self.get_logger().info(f"    QR Result: /gorev/qr_result")

    # ═══════════════════════════════════════════════════════════════
    # ROS IMAGE → OPENCV (cv_bridge olmadan)
    # ═══════════════════════════════════════════════════════════════
    def _imgmsg_to_cv2(self, msg):
        """ROS Image mesajını OpenCV BGR formatına çevir (cv_bridge gerektirmez)."""
        dtype = np.uint8
        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif msg.encoding == 'mono8':
            channels = 1
        elif msg.encoding in ('rgba8', 'bgra8'):
            channels = 4
        else:
            channels = 3  # Varsayılan

        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

        # RGB → BGR dönüşümü (OpenCV BGR kullanır)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'rgba8':
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == 'bgra8':
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif msg.encoding == 'mono8':
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        return img

    # ═══════════════════════════════════════════════════════════════
    # ANA GÖRÜNTÜ CALLBACK
    # ═══════════════════════════════════════════════════════════════
    def image_callback(self, msg):
        """Kameradan gelen her frame'i işle."""

        # 1. ROS Image → OpenCV
        try:
            frame = self._imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Frame dönüşüm hatası: {e}")
            return

        # Frame boyutlarını güncelle
        h_org, w_org = frame.shape[:2]
        if w_org != self.frame_width or h_org != self.frame_height:
            self.frame_width = w_org
            self.frame_height = h_org
            self.frame_center = (w_org // 2, h_org // 2)

        # FPS
        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time) if (curr_time - self.prev_time) > 0 else 0
        self.prev_time = curr_time

        # ═══ QR Sonuç Kontrolü ═══
        try:
            res = self.qr_result_queue.get_nowait()
            if res:
                if self.locked_qr is None:
                    self.locked_qr = res
                    self.get_logger().info(f"🔥 QR KİLİTLENDİ: {res}")
                    self.qr_control_queue.put("PAUSE")

                    # QR metnini ROS topic'e yayınla
                    qr_msg = String()
                    qr_msg.data = str(res)
                    self.qr_text_pub.publish(qr_msg)
        except Exception:
            pass

        # ═══ DURUM MAKİNESİ ═══
        detected_box_global = None

        if self.state == "SEARCH":
            detected_box_global = self._process_search(frame, w_org, h_org)

        elif self.state == "TRACK":
            detected_box_global = self._process_track(frame, w_org, h_org)

        # ═══ TargetInfo YAYINLA ═══
        self._publish_target_info(detected_box_global)

        # ═══ GÖRSELLEŞTİRME ═══
        self._visualize(frame, fps)

    # ═══════════════════════════════════════════════════════════════
    # SEARCH MODU
    # ═══════════════════════════════════════════════════════════════
    def _process_search(self, frame, w_org, h_org):
        """Tüm kare üzerinde YOLO ile hedef ara."""

        # Lock'u temizle (hedef kaybedilip SEARCH'e dönüldü)
        if self.locked_qr is not None:
            self.locked_qr = None
            self.qr_control_queue.put("RESUME")
            self.get_logger().warn("⚠️ Lock Temizlendi (Hedef Kaybedildi)")

        # Hız için resize
        scale = SEARCH_SIZE / w_org
        h_search = int(h_org * scale)
        frame_input = cv2.resize(frame, (SEARCH_SIZE, h_search))

        # YOLO çıkarımı
        results = self.model(frame_input, verbose=False, stream=True, imgsz=SEARCH_SIZE)

        best_conf = 0
        detected_box_global = None

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > CONF_THRESHOLD and conf > best_conf:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    gx1 = int(x1 / scale)
                    gy1 = int(y1 / scale)
                    gx2 = int(x2 / scale)
                    gy2 = int(y2 / scale)
                    detected_box_global = [gx1, gy1, gx2 - gx1, gy2 - gy1]
                    best_conf = conf

        if detected_box_global:
            self.state = "TRACK"
            self.tracker.init(detected_box_global)
            self.last_known_box = detected_box_global
            self.missed_count = 0
            self.get_logger().info("✅ Hedef Bulundu! → TRACK moduna geçildi.")

        return detected_box_global

    # ═══════════════════════════════════════════════════════════════
    # TRACK MODU
    # ═══════════════════════════════════════════════════════════════
    def _process_track(self, frame, w_org, h_org):
        """ROI üzerinde YOLO + Kalman ile hedef takibi + QR okuma."""

        # 1. Kalman tahmini
        pred_box = self.tracker.predict()
        if pred_box is None:
            self.state = "SEARCH"
            return None

        px, py, pw, ph = pred_box
        pcx, pcy = px + pw // 2, py + ph // 2

        # 2. Dinamik ROI (kayıp frame sayısına göre genişlet)
        expansion = 1.0 + (self.missed_count * 0.05)
        roi_w = int(TRACK_ROI_SIZE * expansion)
        roi_h = int(TRACK_ROI_SIZE * expansion)

        rx1 = max(0, pcx - roi_w // 2)
        ry1 = max(0, pcy - roi_h // 2)
        rx2 = min(w_org, rx1 + roi_w)
        ry2 = min(h_org, ry1 + roi_h)
        rx1 = max(0, rx2 - roi_w)
        ry1 = max(0, ry2 - roi_h)

        roi_frame = frame[ry1:ry2, rx1:rx2]
        if roi_frame.size == 0:
            self.state = "SEARCH"
            return None

        # 3. ROI üzerinde YOLO
        results = self.model(roi_frame, verbose=False, stream=True, imgsz=TRACK_ROI_SIZE)

        found_in_roi = False
        best_conf = 0
        detected_box_global = None

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > CONF_THRESHOLD and conf > best_conf:
                    bx1, by1, bx2, by2 = box.xyxy[0].cpu().numpy()
                    detected_box_global = [
                        int(rx1 + bx1), int(ry1 + by1),
                        int(bx2 - bx1), int(by2 - by1)
                    ]
                    found_in_roi = True
                    best_conf = conf

        if found_in_roi:
            self.last_known_box = self.tracker.update(detected_box_global)
            self.missed_count = 0

            # QR Okuma (kilitli değilse)
            if self.locked_qr is None:
                pad_x = int(detected_box_global[2] * QR_PADDING)
                pad_y = int(detected_box_global[3] * QR_PADDING)

                qx1 = max(0, detected_box_global[0] - pad_x)
                qy1 = max(0, detected_box_global[1] - pad_y)
                qx2 = min(w_org, detected_box_global[0] + detected_box_global[2] + pad_x)
                qy2 = min(h_org, detected_box_global[1] + detected_box_global[3] + pad_y)

                qr_crop = frame[qy1:qy2, qx1:qx2]
                if self.qr_reader.is_alive():
                    self.qr_reader.add_frame(qr_crop)
        else:
            self.missed_count += 1
            self.last_known_box = pred_box
            if self.missed_count > LOST_THRESHOLD:
                self.get_logger().warn("⚠️ Hedef Kayıp → SEARCH moduna dönüldü.")
                self.state = "SEARCH"
                self.tracker.active = False
                if self.locked_qr:
                    self.locked_qr = None
                    self.qr_control_queue.put("RESUME")

        return detected_box_global

    # ═══════════════════════════════════════════════════════════════
    # TargetInfo YAYINLAMA
    # ═══════════════════════════════════════════════════════════════
    def _publish_target_info(self, detected_box):
        """Hedef bilgisini TargetInfo mesajı olarak yayınla."""
        target_msg = TargetInfo()

        if self.state == "TRACK" and self.last_known_box is not None:
            lx, ly, lw, lh = self.last_known_box
            cx = lx + lw // 2
            cy = ly + lh // 2

            target_msg.detected = True
            target_msg.target_x = float(cx)
            target_msg.target_y = float(cy)
            target_msg.target_z = 0.0
            target_msg.confidence = 1.0 if self.locked_qr else 0.8
            target_msg.bbox_cx = float(cx)
            target_msg.bbox_cy = float(cy)
            target_msg.bbox_w = float(lw)
            target_msg.bbox_h = float(lh)
            target_msg.target_class = "qr_target"
            target_msg.target_id = 1

            # Piksel hatası (kamera merkezinden sapma)
            target_msg.error_x = float(cx - self.frame_center[0])
            target_msg.error_y = float(cy - self.frame_center[1])
        else:
            target_msg.detected = False
            target_msg.target_class = "qr_target"
            target_msg.error_x = 0.0
            target_msg.error_y = 0.0

        self.target_pub.publish(target_msg)

    # ═══════════════════════════════════════════════════════════════
    # GÖRSELLEŞTİRME (cv2.imshow)
    # ═══════════════════════════════════════════════════════════════
    def _visualize(self, frame, fps):
        """Hedef kutusu, FPS, mod ve QR durumunu ekrana çiz."""
        vis_frame = frame.copy()

        if self.last_known_box and self.state == "TRACK":
            lx, ly, lw, lh = self.last_known_box

            if self.locked_qr:
                color = (0, 255, 0)       # Yeşil: QR Kilitli
            elif self.missed_count == 0:
                color = (255, 255, 0)     # Cyan: Takip ediliyor
            else:
                color = (0, 0, 255)       # Kırmızı: Kayıp/Tahmin

            cv2.rectangle(vis_frame, (lx, ly), (lx + lw, ly + lh), color, 3)

            if self.locked_qr:
                cv2.putText(vis_frame, str(self.locked_qr),
                            (lx, ly - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Bilgi metinleri
        cv2.putText(vis_frame, f"FPS: {int(fps)}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(vis_frame, f"MODE: {self.state}",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        if self.locked_qr:
            cv2.putText(vis_frame, "STATUS: LOCKED",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Ekrana göster
        vis_small = cv2.resize(vis_frame, (1280, 720))
        cv2.imshow(self.win_name, vis_small)
        cv2.waitKey(1)

    # ═══════════════════════════════════════════════════════════════
    # KAPATMA
    # ═══════════════════════════════════════════════════════════════
    def destroy_node(self):
        """Temiz kapatma."""
        self.get_logger().info("🛑 QR Vision Node kapatılıyor...")
        try:
            self.qr_control_queue.put("STOP")
            self.qr_reader.join(timeout=1.0)
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    multiprocessing.set_start_method('spawn', force=True)
    rclpy.init(args=args)
    node = QRVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
