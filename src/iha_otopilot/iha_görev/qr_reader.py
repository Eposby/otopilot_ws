"""
qr_reader.py — Asenkron QR Kod Okuyucu (Multiprocessing)
WeChat QR Code kütüphanesi ile çoklu görüntü iyileştirme pipeline'ı kullanarak
QR kod çözme işlemi yapar. Ana işlemi yavaşlatmamak için ayrı süreçte çalışır.
"""

import os
import cv2
import multiprocessing
import queue
import time
import numpy as np
from iha_görev import qr_utils

# Model dosyalarının bulunduğu dizin (bu dosyanın yanındaki models/ klasörü)
_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')

class AsyncQRReader(multiprocessing.Process):
    def __init__(self, result_queue, control_queue):
        super().__init__()
        self.frame_queue = multiprocessing.Queue(maxsize=1)
        self.result_queue = result_queue
        self.control_queue = control_queue # To send 'pause', 'stop' commands
        self.daemon = True # Kill if main dies

    def add_frame(self, crop_img):
        try:
            # Non-blocking put with drop-oldest strategy
            self.frame_queue.put_nowait(crop_img)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
                self.frame_queue.put_nowait(crop_img)
            except queue.Empty:
                pass

    def _unsharp_mask(self, image, sigma=1.0, amount=1.5):
        blurred = cv2.GaussianBlur(image, (0, 0), sigma)
        sharpened = cv2.addWeighted(image, 1.0 + amount, blurred, -amount, 0)
        return sharpened

    def _is_blurry(self, image, threshold=100.0):
        # Laplacian Variance method
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        score = cv2.Laplacian(gray, cv2.CV_64F).var()
        return score < threshold

    def run(self):
        # --- INITIALIZATION INSIDE PROCESS ---
        # OpenCV objects cannot be pickled, so we init here.
        print("🚀 QR Process: Starting...")

        if not qr_utils.check_and_download_models(_MODELS_DIR):
            print("❌ QR Process: Model files missing.")
            return

        # Model dosya yolları
        det_proto = os.path.join(_MODELS_DIR, 'detect.prototxt')
        det_model = os.path.join(_MODELS_DIR, 'detect.caffemodel')
        sr_proto  = os.path.join(_MODELS_DIR, 'sr.prototxt')
        sr_model  = os.path.join(_MODELS_DIR, 'sr.caffemodel')

        detector = None
        try:
            if hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
                 detector = cv2.wechat_qrcode_WeChatQRCode(
                    det_proto, det_model, sr_proto, sr_model
                )
            elif hasattr(cv2, 'wechat_qrcode') and hasattr(cv2.wechat_qrcode, 'WeChatQRCode'):
                detector = cv2.wechat_qrcode.WeChatQRCode(
                    det_proto, det_model, sr_proto, sr_model
                )
            else:
                print("❌ QR Process: WeChatQRCode not found.")
        except Exception as e:
            print(f"❌ QR Process: Init Failed: {e}")

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        paused = False

        print("✅ QR Process: Ready.")

        while True:
            # 1. Check Control Messages
            try:
                msg = self.control_queue.get_nowait()
                if msg == "STOP":
                    break
                elif msg == "PAUSE":
                    paused = True
                    # Clear queue
                    while not self.frame_queue.empty():
                        try: self.frame_queue.get_nowait()
                        except: pass
                elif msg == "RESUME":
                    paused = False
            except queue.Empty:
                pass

            if paused:
                time.sleep(0.05)
                continue

            # 2. Get Frame
            try:
                frame = self.frame_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            if detector is None: continue

            try:
                # 1. AKTİF: Bulanıklık Kontrolü (Blur Check) Eklentisi
                # Resim çok bulanıksa ağır işlemleri yapıp CPU'yu yormadan direkt atlar.
                # Threshold değerini 30 olarak belirliyoruz (sadece çok bulanıkları eler)
                if self._is_blurry(frame, threshold=30.0):
                    continue

                found_qr = None
                
                # --- ÇOK AŞAMALI (MULTI-STAGE) YÜKSEK İRTİFA QR OKUMA PIPELINE'I ---

                # A. Raw (Ham Görüntü) Okuma
                res, _ = detector.detectAndDecode(frame)
                if len(res) > 0 and len(res[0]) > 0:
                    found_qr = res[0]

                # B. PERSPEKTİF DÜZELTME EKLENTİSİ (Açılı Kamerayı Düze Çevir)
                # Uçak açılı yaklaşıyorsa QR kodu yamuk (Trapezoid) görünür. 
                # Standart OpenCV dedektörü ile 4 köşeyi arayıp homography ile resmi düze çekeriz.
                if not found_qr:
                    cv2_qr = cv2.QRCodeDetector()
                    ret, points = cv2_qr.detect(frame)
                    if ret and points is not None:
                        pts = points[0]
                        w_warp, h_warp = 300, 300 # İdeal okuma boyutu
                        pts_dst = np.array([
                            [0, 0],
                            [w_warp - 1, 0],
                            [w_warp - 1, h_warp - 1],
                            [0, h_warp - 1]
                        ], dtype=np.float32)
                        
                        matrix = cv2.getPerspectiveTransform(pts, pts_dst)
                        warped = cv2.warpPerspective(frame, matrix, (w_warp, h_warp))
                        
                        res, _ = detector.detectAndDecode(warped)
                        if len(res) > 0 and len(res[0]) > 0:
                            found_qr = res[0]

                # C. CLAHE (Kontrast ve Parlaklık Düzeltme)
                if not found_qr:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
                    enhanced = clahe.apply(gray)
                    input_2 = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

                    res, _ = detector.detectAndDecode(input_2)
                    if len(res) > 0 and len(res[0]) > 0:
                        found_qr = res[0]

                # D. ÇOKLU ÖLÇEK (MULTI-SCALE) & MORFOLOJİK İŞLEMLER EKLENTİSİ
                if not found_qr:
                    if 'enhanced' not in locals():
                         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
                         enhanced = clahe.apply(gray)

                    # Multi-scale denemeleri: İHA'nın yüksekliğine göre farklı ölçeklerde (1.5x, 2.0x, 3.0x) büyütmeyi deneriz.
                    scales = [1.5, 2.0, 3.0]
                    for scale in scales:
                        zoomed = cv2.resize(enhanced, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
                        sharpened = self._unsharp_mask(zoomed, sigma=1.0, amount=1.5)
                        input_3 = cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)

                        res, _ = detector.detectAndDecode(input_3)
                        if len(res) > 0 and len(res[0]) > 0:
                            found_qr = res[0]
                            break # Bulduğumuz an zaman kaybetmeden döngüden çık
                        
                        # Hala bulamadıysa (Çok kötü/Puslu Işık): Adaptive Threshold ve Morfolojik Filtreler!
                        if not found_qr:
                            thresh = cv2.adaptiveThreshold(sharpened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                            
                            # YENİ: Morfolojik Temizlik (Kapanış İşlemi) Eklentisi
                            # Bu işlem QR kod üzerindeki beyaz kireçlenmeleri ve güneş parazitlerini doldurur, pikselleri bütünleştirir.
                            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                            cleaned = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
                            
                            input_4 = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                            res, _ = detector.detectAndDecode(input_4)
                            if len(res) > 0 and len(res[0]) > 0:
                                found_qr = res[0]
                                break

                # Eğer herhangi bir aşamada hedef başarıyla bulunduysa merkeze bildir ve uykuya geç
                if found_qr:
                    self.result_queue.put(found_qr)
                    paused = True 

            except Exception as e:
                # print(f"QR Process Error: {e}")
                pass

        print("🛑 QR Process: Stopped.")






    # def run(self):
    #     # ... (modeller yüklenir vs.)
        
    #     while self._is_running.value:
    #         # ...
    #         try:
    #             frame = self.frame_queue.get(timeout=0.1)
                
    #             # ======= SENİN İŞLEMLERİNİ EKLEYECEĞİN YER BURASI =======
    #             # Gelen 'frame' sadece QR kodun olduğu küçük hedeftir.
                
    #             # Örnek 1: Resmi Griye Çevirme
    #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
    #             # Örnek 2: Otsu Thresholding ile gereksiz arkaplanı silip siyah/beyaz yapma
    #             _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                
    #             # Örnek 3: Parazitleri silme (Blur/Median)
    #             # blur = cv2.medianBlur(thresh, 3)
                
    #             # İşlenmiş resmi QR okuyucuya veriyoruz (Bazen orijinali de okutmak iyi olabilir, ikisini de deneriz)
    #             res, _ = detector.detectAndDecode(thresh) 
    #             # ========================================================
                
    #             # ...
