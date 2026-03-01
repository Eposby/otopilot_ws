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


qr_vision_node.py: sistemin çalışması için gerekli parametreleri içerir. bunlar SEARCH_SIZE, TRACK_ROI_SIZE, CONF_THRESHOLD, LOST_THRESHOLD, QR_PADDING
                    SEARCH_SIZE: arama modunda resim boyutu
                    TRACK_ROI_SIZE: takip modunda ROI boyutu
                    CONF_THRESHOLD: YOLO güven eşiği
                    LOST_THRESHOLD: kaçılan frame sayısını belirler
                    QR_PADDING: QR crop çerçevesine padding oranı



qr_tracker.py: Hedef (uçak/QR) tespit edildikten sonra, hedef anlık olarak kameradan çıksa veya kaybolsa bile nereye gidebileceğini matematiksel olarak tahmin eden (Kalman Filtresi) koddur.
qr_reader.py: Ayrı bir işlemci çekirdeğinde (multiprocessing) arka planda çalışarak, hedefe yaklaşıldığında QR kodun içeriğini (örneğin "hedef_koordinat") okuyan koddur.

gz_bridge_camera.py: Gazebo kamerasından gelen görüntüyü ROS 2 topic'ine yayınlayan koddur.Gazebo simülasyonundaki (veya gerçekteki) kameradan ham görüntüyü alıp 
qr_vision_node.py'ye saniyede 30-60 defa gönderen koddur. Eğer bu kod çalışmıyorsa vision node siyah ekran verir.


qr2.py:  (Asıl Uçuş Kontrolcüsü): Uçağın hedefe doğru dalış yapmasını sağlayan "Kamikaze" koddur. Vizyondan gelen hedef merkezden sapma (error_x, error_y) değerlerini okur ve uçağın kanatçıklarına sinyal göndererek hedefi tam ortalamasını sağlar.

bu sistemin veri akışı şöyledir qr2.py en sonda çünkü uçak kalkmadan da kamera anlık olarak veri okur ancak qr_vision_node ile aldığımız veri sayesinde anlık olarak hedef merkezini bulup qr2.py'ye gönderiyoruz.
gz_bridge_camera -> qr_vision_node -> qr2.py

"""

import os
import time
import multiprocessing
# Çoklu işlemci kullanımı. QR okuma ağır bir işlemdir, videoyu kastırmasın ayrı bir çekirdekte (arkaplanda) çalışsın diye kullanıyoruz.


import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
# ROS'un standart "Fotoğraf" mesaj tipi. Kameradan gelen sinyali bu formatta alıyoruz.


from std_msgs.msg import String
# ROS'un standart "Metin" mesaj tipi. Okunan QR kodunu (örneğin "Koordinat: 41, 29") yazılı olarak göndermek için.

# cv_bridge yerine manuel dönüşüm (NumPy versiyon uyumsuzluğunu önler)
# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# Custom mesaj
# Sadece bizim projeye özel yazdığımız sinyal dili. (İçinde hedef_x, error_y, bbox_w gibi uçağı uçuracak özel veriler var).
from iha_messages.msg import TargetInfo

# Aynı paketteki destek modülleri
# Kamera anlık kör olsa (hedef çıksa) bile hedefin nereye gittiğini tahmin eden matematiksel kodumuz (Diğer dosyadan çekiyoruz).
from iha_görev.qr_tracker import KalmanBoxTracker

# Arka planda kasılmadan QR okuyan kodumuz.
from iha_görev.qr_reader import AsyncQRReader

# Model dizini
_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')

# Bu satır çok zekice bir iş yapar: "Benim şu an çalıştığım bu Python dosyası (qr_vision_node.py) bilgisayarda hangi klasördeyse, onun yanındaki 
# models klasörüne git." Böylece kodu sen Samet'in bilgisayarından kendi bilgisayarına kopyalasan bile, 
# C:/ veya /home/mert/ yolları değişse bile, kod hedef modellerini (best.pt) eliyle koymuş gibi bulur.


# ═══════════════════════════════════════════════════════════════
# YAPILANDIRMA PARAMETRELERİ
# ═══════════════════════════════════════════════════════════════
SEARCH_SIZE = 640           # SEARCH modunda resize boyutu yolo modelinin taradığı boyut büyük olursa fps düşer
TRACK_ROI_SIZE = 640        # TRACK modunda ROI boyutu takip ederken sadece o kısma yakın 640 boyutunda arama yapar
CONF_THRESHOLD = 0.4        # YOLO güven eşiği
LOST_THRESHOLD = 30         # Kaç frame kaybedilirse SEARCH'e dön 
QR_PADDING = 0.2            # QR crop çerçevesine padding oranı weChat Qr okurken panonun kenarına yaklaşıldığında padding oranı

# ═══════════════════════════════════════════════════════════════
# QR Vision Node

# Yapay zeka uçağı ararken video hiç donmasın (kasıp yavaşlamasın) isteriz. QR kodu okumak ise yavaş bir işlemdir. Bu yüzden bilgisayarın beynini ikiye böleriz (Multiprocessing).
# qr_result_queue: Yan odadaki işçinin (QR okuyucu) bize "Abi QR'ı okudum, metin şu!" diye mesaj yolladığı boru hattıdır.
# qr_control_queue: Senin işçiye "Şimdi çalış" veya "Şimdi bekle" dediğin borudur.
# .start(): Fabrikanın yan odasındaki bu gizli işçiyi çalışmaya başlatır. Beklemeye geçer.



# ═══════════════════════════════════════════════════════════════
class QRVisionNode(Node):

    def __init__(self):
        super().__init__('qr_vision_node')

        # ═══ Durum Kontrolleri ═══
        if not YOLO_AVAILABLE:
            self.get_logger().fatal("ultralytics bulunamadı! 'pip install ultralytics' gerekli.")
            return

        # ═══ YOLO Model Yükleme ═══
        model_path = os.path.join(_MODELS_DIR, 'best.pt') # _MODELS_DIR path ile best.pt birleştirir ve path tam olur
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ YOLO model yüklendi: {model_path}")
        except Exception as e:
            self.get_logger().fatal(f"❌ YOLO model yüklenemedi: {e}")
            return

        # ═══ QR Okuyucu (Multiprocessing) ═══
        # bunların çalışma prensibi için qr_vision_node.py dosyasındaki yorum satırlarını okuyunuz.
        self.qr_result_queue = multiprocessing.Queue()
        self.qr_control_queue = multiprocessing.Queue()
        self.qr_reader = AsyncQRReader(self.qr_result_queue, self.qr_control_queue)
        self.qr_reader.start()

        # ═══ Kalman Tracker ═══
        # bu nesne qr_tracker.py'dan çekildi
        self.tracker = KalmanBoxTracker()

        # ═══ Durum Değişkenleri ═══
        self.state = "SEARCH"       # SEARCH veya TRACK
        self.missed_count = 0       # Hedefi art arda kaç kere göremediğimizi sayar. (LOST_THRESHOLD'u geçerse TRACK biter).
        self.locked_qr = None       # Okunan QR metni (kilitlendi mi?)  # Hedefteki QR kod okunduğunda metin buraya yazılır. None=Henüz okuyamadık.
        self.last_known_box = None   # Son bilinen hedef kutusu [x, y, w, h] # Hedefin ekranda en son nerede görüldüğünün kutusu [X, Y, Genişlik, Yükseklik].
        self.prev_time = time.time()

        # Frame boyutları (ilk frame'de güncellenecek)
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center = (320, 240)

        # ═══ ROS 2 Parametreleri ═══
        #declare_parameter('camera_topic', '/camera/image_raw'): "Ben bu koda dışarıdan müdahale edilebilir bir ayar düğmesi (Parametre) ekliyorum.
        #Adı da camera_topic olsun. Eğer kimse dışarıdan bana aksini söylemezse, varsayılan (default) olarak /camera/image_raw kanalını dinleyeceğim" der.
        

        #self.declare_parameter(): ROS 2'ye "Bana bir ayar düğmesi tanımla" deme komutu.
        #self.get_parameter(): ROS 2'den "O ayar düğmesinin şu anki değeri ne?" diye sorma komutu.

        self.declare_parameter('camera_topic', '/camera/image_raw')
        #get_parameter(...).value: O anki ayar düğmesinin (parametrenin) üstünde hangi değerin yazdığını okuyup camera_topic isimli kelime haznesine (
        #değişkene) atar. Sonra da aşağıdaki dinleyici (Subscriber) fonksiyona "Al bak, bu kamera kanalını dinleyeceksin" diye bu değişkeni verir.
        camera_topic = self.get_parameter('camera_topic').value


        #!!!!
        #ros2 run iha_otopilot qr_vision --ros-args -p camera_topic:=/usb_cam/image

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
    # ROS'tan gelen kameranın nasıl bir resim yolladığını kontrol eder:
    # Renkli resimse (Kırmızı, Yeşil, Mavi) -> 3 kanal (channels = 3) vardır.
    # Siyah/Beyaz gece görüş kamerasıysa (mono) -> Sadece grilik vardır, yani 1 kanal (channels = 1).
    # Arkası saydam bir resimse (rgba) -> 4 kanal vardır.



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
        # Burası fonksiyonun kalbidir. msg.data dediğimiz şey, 
        # milyonlarca anlamsız sayının yan yana dizildiği upuzun bir listedir (
        # Örn: [255, 0, 12, 54, 255...]). Bu liste bilgisayar için hiçbir şey ifade etmez. 
        # reshape(Yükseklik, Genişlik, RenkKanalı) komutu şunu yapar: "Bu upuzun ipliği alıp, örneğin 640x480 boyutunda bir kilim dokur gibi dikdörtgen bir resim haline getir."
        #  Artık elimizde gerçek bir görüntü vardır.

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
        # get_nowait(): Buradaki "no wait" (bekleme yok) kısmı çok kritiktir. Normalde birinden cevap beklerken o cevap gelene kadar donup kalırsın. Ama biz kameradan saniyede 30 tane resim alıyoruz, donup beklemeye vaktimiz yok!
        # Bu komut şunu der: "Hey işçi, telsizde bana yolladığın bir QR sonucu var mı? Varsa hemen ver, yoksa hiç bekletme ben işime (video akıtmaya) devam ediyorum." Eğer telsizde bekleyen bir mesaj yoksa bu komut hata fırlatır, biz de o hatayı aşağıdaki except Exception: pass ile sessizce görmezden geliriz.

        try:
            res = self.qr_result_queue.get_nowait()
            if res:
                if self.locked_qr is None:
                    self.locked_qr = res
                    self.get_logger().info(f"🔥 QR KİLİTLENDİ: {res}")
                    # Ekrana "Kilitlendim" bilgisini yazar.
                    # self.qr_control_queue.put("PAUSE"): Bu çok akıllıca bir enerji tasarrufudur. Zaten QR kodunu 1 kere okuduk ve ne yazdığını (hedefin ne olduğunu) öğrendik. Artık hedefin üzerine inene kadar her saniye o aynı QR kodunu tekrar tekrar okumakla işlemciyi yormaya gerek var mı? Hayır! İşçimize diğer telsizden "PAUSE" (Bekle) komutu göndeririz. İşçi işlemciyi meşgul etmeyi bırakır. (Eğer hedefi kaybedersek ileride işçiye tekrar "RESUME" deyip onu uyandıracağız).
                    self.qr_control_queue.put("PAUSE")

                    # QR metnini ROS topic'e yayınla
                    qr_msg = String()
                    qr_msg.data = str(res)
                    self.qr_text_pub.publish(qr_msg)
                    print(f"QR KİLİTLENDİ: {res}")
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
        # cv2.resize ile gelen frame (fotoğraf) yeni boyutlara sıkıştırılır.\n        
        # Yeni boyutlar (Genişlik=SEARCH_SIZE=640, Yükseklik=h_search=360 vb.) şeklindedir.\n        

        # Sen kodun en başında SEARCH_SIZE = 640 diye bir kural koymuştun ("Ben resmi 640 genişliğe küçültmek istiyorum" demiştin). Burada bilgisayar soruyor: "1920'yi kaça bölersem veya neyle çarparsam 640 olur?"
        # scale = 640 / 1920
        # scale = 0.33 (Yani resmin 3'te 1'i kadar küçülmesi lazım)
        

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

            # Gördüğümüz bu hedefin koordinatlarını last_known_box (Bilinen Son Kutu) isimli hafızamıza kaydederiz. 
            # Uçuş kontrolcüsü (qr2.py - Kamikaze kodumuz) uçağı kanatçıklarla yönlendirirken "Bana hedefin son görüldüğü yeri verin" 
            # derse diye cebimizde tutarız.
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
