# qr_reader.py: Asenkron QR kod okuma işlemini gerçekleştirir. OpenCV'nin wechat_qrcode (WeChat QR) kütüphanesini kullanarak, 
# main.py'nin tespit edip gönderdiği küçük resim kesitleri üzerinde (görüntüyü keskinleştirerek, zıtlığı artırarak vs.) 
# QR kodu çözmeye çalışır. İşlem ana akışı yavaşlatmasın diye multiprocessing ile ayrı bir süreçte çalışır.



import cv2
import multiprocessing
import queue
import time
import numpy as np
import utils

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

        if not utils.check_and_download_models():
            print("❌ QR Process: Model files missing.")
            return

        detector = None
        try:
            if hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
                 detector = cv2.wechat_qrcode_WeChatQRCode(
                    "detect.prototxt", "detect.caffemodel",
                    "sr.prototxt", "sr.caffemodel"
                )
            elif hasattr(cv2, 'wechat_qrcode') and hasattr(cv2.wechat_qrcode, 'WeChatQRCode'):
                detector = cv2.wechat_qrcode.WeChatQRCode(
                    "detect.prototxt", "detect.caffemodel",
                    "sr.prototxt", "sr.caffemodel"
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
                # 3. Blur Check (Efficiency)
                # If frame is too blurry, skip heavy processing
                # Threshold logic: small images naturally have lower variance,
                # so we adjust threshold or just be lenient.
                # For now, let's just log or be very conservative (e.g. 50).
                # Skipping this for now to ensure robustness, or maybe just log it.
                # if self._is_blurry(frame, threshold=50.0):
                #     continue

                found_qr = None

                # --- PIPELINE ---

                # A. Raw
                res, _ = detector.detectAndDecode(frame)
                if len(res) > 0 and len(res[0]) > 0:
                    found_qr = res[0]

                # B. CLAHE
                if not found_qr:
                    if len(frame.shape) == 3:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    else:
                        gray = frame
                    enhanced = clahe.apply(gray)
                    input_2 = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

                    res, _ = detector.detectAndDecode(input_2)
                    if len(res) > 0 and len(res[0]) > 0:
                        found_qr = res[0]

                # C. Turbo (Upscale + Sharpen)
                if not found_qr:
                    # Reuse enhanced
                    if 'enhanced' not in locals():
                         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                         enhanced = clahe.apply(gray)

                    # Upscale 2x
                    zoomed = cv2.resize(enhanced, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                    sharpened = self._unsharp_mask(zoomed, sigma=1.0, amount=1.5)
                    input_3 = cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)

                    res, _ = detector.detectAndDecode(input_3)
                    if len(res) > 0 and len(res[0]) > 0:
                        found_qr = res[0]
                    else:
                        # D. Threshold
                        thresh = cv2.adaptiveThreshold(sharpened, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                        input_4 = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                        res, _ = detector.detectAndDecode(input_4)
                        if len(res) > 0 and len(res[0]) > 0:
                            found_qr = res[0]

                if found_qr:
                    self.result_queue.put(found_qr)
                    paused = True # Auto-pause locally to save CPU immediately?
                    # No, let main controller decide logic.

            except Exception as e:
                # print(f"QR Process Error: {e}")
                pass

        print("🛑 QR Process: Stopped.")
