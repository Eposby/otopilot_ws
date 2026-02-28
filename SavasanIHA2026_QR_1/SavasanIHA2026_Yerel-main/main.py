# main.py: Sistemin ana çalışma dosyasıdır (Main program). Kameradan veya bir video dosyasından görüntüleri alır. 
# YOLOv8 modelini kullanarak ekrandaki hedefi arar (SEARCH). Hedef bulunduğunda, bunu kare içine alır ve takip moduna geçer (TRACK). 
# Takip ettiği alanın küçük bir kesitini QR kod okunması için qr_reader.py sürecine (process) gönderir. 
# Ekrana fps, mod ve hedef bulma durumu gibi görsel çıktıları bastırır.




import cv2
import time
import numpy as np
import argparse
import multiprocessing
from ultralytics import YOLO
from tracker import KalmanBoxTracker
from qr_reader import AsyncQRReader
import utils

# --- CONFIGURATION ---
SEARCH_SIZE = 640
TRACK_ROI_SIZE = 640
CONF_THRESHOLD = 0.4
LOST_THRESHOLD = 30
QR_PADDING = 0.2

def main():
    parser = argparse.ArgumentParser(description="Kamikaze UAV QR System")
    parser.add_argument("--source", type=str, default="QRTest_video1.mp4", help="Video file or 'camera'")
    parser.add_argument("--model", type=str, default="best.pt", help="YOLO model path (.pt or .engine)")
    args = parser.parse_args()

    print("🚀 SYSTEM INITIALIZING...")

    # 1. Load Model
    try:
        model = YOLO(args.model)
        print(f"✅ Model loaded: {args.model}")
    except Exception as e:
        print(f"❌ Model load error: {e}")
        return

    # 2. Init Subsystems (Multiprocessing)
    result_queue = multiprocessing.Queue()
    control_queue = multiprocessing.Queue()

    qr_reader = AsyncQRReader(result_queue, control_queue)
    qr_reader.start()

    # 3. Video Capture
    if args.source == "camera":
        # Try GStreamer for Jetson
        pipeline = utils.get_gstreamer_pipeline()
        print(f"📷 Attempting GStreamer: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("⚠️ GStreamer failed. Falling back to V4L2 index 0...")
            cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(args.source)

    if not cap.isOpened():
        print("❌ Video source not found.")
        qr_reader.terminate()
        return

    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"📷 Input Resolution: {fw}x{fh}")

    # Display Window
    win_name = "KAMIKAZE QR SYSTEM"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    # State Variables
    state = "SEARCH" # SEARCH, TRACK
    missed_count = 0
    locked_qr = None

    # For FPS calculation
    prev_time = time.time()
    last_known_box = None
    tracker = KalmanBoxTracker()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("End of stream.")
                break

            # FPS Tick
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            h_org, w_org = frame.shape[:2]

            # --- LOCK MECHANISM & RESULT CHECK ---
            # Check for new results from process
            try:
                res = result_queue.get_nowait()
                if res:
                    if locked_qr is None:
                        locked_qr = res
                        print(f"🔥 QR LOCKED: {res}")
                        control_queue.put("PAUSE") # Optim: Stop reader
            except:
                pass

            # --- STATE MACHINE ---

            detected_box_global = None

            if state == "SEARCH":
                # Reset Lock if we lost target and went back to search?
                if locked_qr is not None:
                     locked_qr = None
                     control_queue.put("RESUME")
                     print("⚠️ Lock Cleared (Target Lost)")

                # 1. Resize for speed
                scale = SEARCH_SIZE / w_org
                h_search = int(h_org * scale)
                frame_input = cv2.resize(frame, (SEARCH_SIZE, h_search))

                # 2. Inference
                results = model(frame_input, verbose=False, stream=True, imgsz=SEARCH_SIZE)

                best_conf = 0
                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf > CONF_THRESHOLD and conf > best_conf:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            # Convert to global
                            gx1 = int(x1 / scale)
                            gy1 = int(y1 / scale)
                            gx2 = int(x2 / scale)
                            gy2 = int(y2 / scale)
                            detected_box_global = [gx1, gy1, gx2 - gx1, gy2 - gy1]
                            best_conf = conf

                if detected_box_global:
                    state = "TRACK"
                    tracker.init(detected_box_global)
                    last_known_box = detected_box_global
                    missed_count = 0
                    print("✅ Target Acquired! Switching to TRACK mode.")

            elif state == "TRACK":
                # 1. Kalman Predict
                pred_box = tracker.predict()
                if pred_box is None:
                    state = "SEARCH"
                    continue

                px, py, pw, ph = pred_box
                pcx, pcy = px + pw//2, py + ph//2

                # 2. Dynamic ROI
                # Optimization for High Speed:
                # If we missed it recently, expand the search ROI slightly to catch fast moves
                expansion = 1.0 + (missed_count * 0.05) # 5% larger per missed frame
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
                    state = "SEARCH"
                    continue

                # 3. Inference on ROI
                # Note: Model expects square inputs ideally, but YOLO handles rectangles.
                # However, resizing to model's imgsz (640) is key.
                results = model(roi_frame, verbose=False, stream=True, imgsz=TRACK_ROI_SIZE)

                found_in_roi = False
                best_conf = 0

                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf > CONF_THRESHOLD and conf > best_conf:
                            bx1, by1, bx2, by2 = box.xyxy[0].cpu().numpy()
                            detected_box_global = [int(rx1+bx1), int(ry1+by1), int(bx2-bx1), int(by2-by1)]
                            found_in_roi = True
                            best_conf = conf

                if found_in_roi:
                    last_known_box = tracker.update(detected_box_global)
                    missed_count = 0

                    # --- QR READING (Only if NOT locked) ---
                    if locked_qr is None:
                        pad_x = int(detected_box_global[2] * QR_PADDING)
                        pad_y = int(detected_box_global[3] * QR_PADDING)

                        qx1 = max(0, detected_box_global[0] - pad_x)
                        qy1 = max(0, detected_box_global[1] - pad_y)
                        qx2 = min(w_org, detected_box_global[0] + detected_box_global[2] + pad_x)
                        qy2 = min(h_org, detected_box_global[1] + detected_box_global[3] + pad_y)

                        qr_crop = frame[qy1:qy2, qx1:qx2]

                        # Add to Process Queue
                        # Process handles queue full/dropping automatically
                        if qr_reader.is_alive():
                            qr_reader.add_frame(qr_crop)

                else:
                    missed_count += 1
                    last_known_box = pred_box
                    if missed_count > LOST_THRESHOLD:
                        print("⚠️ Target Lost. Switching to SEARCH.")
                        state = "SEARCH"
                        tracker.active = False
                        # Ensure lock is cleared if we lose track completely
                        if locked_qr:
                            locked_qr = None
                            control_queue.put("RESUME")

            # --- VISUALIZATION ---
            vis_frame = frame

            if last_known_box and state == "TRACK":
                lx, ly, lw, lh = last_known_box
                # Green if Locked, Blue if Tracking, Red if Lost/Predicting
                if locked_qr:
                    color = (0, 255, 0)
                elif missed_count == 0:
                    color = (255, 255, 0)
                else:
                    color = (0, 0, 255)

                cv2.rectangle(vis_frame, (lx, ly), (lx+lw, ly+lh), color, 3)

                if locked_qr:
                    cv2.putText(vis_frame, str(locked_qr), (lx, ly - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            cv2.putText(vis_frame, f"FPS: {int(fps)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(vis_frame, f"MODE: {state}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

            if locked_qr:
                cv2.putText(vis_frame, "STATUS: LOCKED", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            vis_small = cv2.resize(vis_frame, (1280, 720))
            cv2.imshow(win_name, vis_small)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("🛑 Shutting down...")
        control_queue.put("STOP")
        qr_reader.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True) # Safe for OpenCV
    main()
