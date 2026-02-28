# tracker.py: Hedefin kamerada kaybolduğu saniyelerde veya anlık olarak görünmediğinde konumunu tahmin edebilmek için
# bir Kalman Filtresi (KalmanBoxTracker) kullanır. YOLO hedefi bir anlığına kaçırsa bile, hedefin hangi yöne hareket edeceğini 
# önceki hız ve konumuna bakarak hesaplayarak kameranın hedefi kaybetmemesini sağlar.


import cv2
import numpy as np

class KalmanBoxTracker:
    def __init__(self):
        # 8 States: [x, y, w, h, vx, vy, vw, vh]
        # 4 Measurements: [x, y, w, h]
        self.kf = cv2.KalmanFilter(8, 4)

        # State Transition Matrix (F)
        # x = x + vx * dt
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ], np.float32)

        # Measurement Matrix (H)
        # We measure x, y, w, h
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], np.float32)

        # Process Noise Covariance (Q)
        # Adjust these to trust model prediction (low noise) or allow flexibility
        self.kf.processNoiseCov = np.eye(8, dtype=np.float32) * 0.03

        # Measurement Noise Covariance (R)
        # Trust measurements fairly well
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.1 # 1e-1

        # Error Covariance (P)
        self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 1.0

        self.last_prediction = None
        self.active = False
        self.missed_frames = 0

    def init(self, box):
        """
        Initialize the filter with the first detection.
        box: [x, y, w, h]
        """
        x, y, w, h = box
        # Initialize state with zero velocity
        self.kf.statePost = np.array([[x], [y], [w], [h], [0], [0], [0], [0]], dtype=np.float32)
        self.active = True
        self.missed_frames = 0
        self.last_prediction = box

    def update(self, box):
        """
        Update the filter with a new measurement (YOLO detection).
        box: [x, y, w, h]
        """
        if not self.active:
            self.init(box)
            return box

        measurement = np.array([[box[0]], [box[1]], [box[2]], [box[3]]], dtype=np.float32)
        self.kf.correct(measurement)

        # We also predict after update to get the best estimate for current frame
        # However, typically you predict() then correct().
        # But for display, we want the corrected state.

        # Let's rely on predict() for the next frame, but return current corrected state
        corrected = self.kf.statePost
        self.active = True
        self.missed_frames = 0

        # corrected is (8, 1) matrix
        res = [int(corrected[0][0]), int(corrected[1][0]), int(corrected[2][0]), int(corrected[3][0])]
        self.last_prediction = res
        return res

    def predict(self):
        """
        Predict the next state. Used when YOLO misses a frame.
        """
        if not self.active:
            return None

        prediction = self.kf.predict()
        # prediction is (8, 1) matrix
        res = [int(prediction[0][0]), int(prediction[1][0]), int(prediction[2][0]), int(prediction[3][0])]

        self.last_prediction = res
        self.missed_frames += 1
        return res
