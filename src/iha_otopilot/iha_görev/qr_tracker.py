# tracker.py: Hedefin kamerada kaybolduğu saniyelerde veya anlık olarak görünmediğinde konumunu tahmin edebilmek için
# bir Kalman Filtresi (KalmanBoxTracker) kullanır. YOLO hedefi bir anlığına kaçırsa bile, hedefin hangi yöne hareket edeceğini 
# önceki hız ve konumuna bakarak hesaplayarak kameranın hedefi kaybetmemesini sağlar.


#===============================================
#Kalman Filtresi, dünyayı iki farklı pencereden görür: 
# Gerçekte ne olduğu (Durum/State) ve bizim sensörle ne gördüğümüz 
# (Ölçüm/Measurement).8 States (Durum): Sistemin içinde tuttuğu, 
# hedefin gerçekte sahip olduğu düşünülen 8 parametredir.$x, y$: 
# Hedef kutusunun merkez koordinatları.$w, h$: Kutusunun genişliği (
# width) ve yüksekliği (height).$vx, vy$: Hedefin X ve Y eksenindeki hızı (
# velocity).$vw, vh$: Hedef kutusunun büyüme/küçülme hızı (Uçak bize yaklaşıyorsa 
# kutu büyür, uzaklaşıyorsa küçülür).4 Measurements (Ölçüm): Kameradan (YOLO'dan) 
# gelen veridir. YOLO bize hedefin hızını veremez! Sadece ekrandaki kutunun 
# koordinatlarını ve boyutlarını verir: $[x, y, w, h]$.




#İşte geleceği tahmin ettiğimiz (Predict) denklem burasıdır. Bu matris, 
#fizik derslerindeki temel hareket denklemi olan $X_{yeni} = X_{eski} + V \cdot \Delta t$ 
#(Yeni Konum = Eski Konum + Hız * Zaman) formülünün matris halidir.Kodda $\Delta t$ (zaman adımı) 1
#olarak kabul edilmiştir (Kareler arası süre sabit varsayılmış).Birinci satıra dikkatlice bak: [1, 0, 0, 0, 1, 0, 0, 0]. 
#Bu satır 8 elemanlı State vektörüyle ($[x, y, w, h, vx, vy, vw, vh]^T$) çarpıldığında şu sonucu üretir:
#Yani: Yeni X = Eski X + X Hızı. Aynı şekilde matrisin alt kısımları, hızların ($vx, vy$) bir sonraki karede de aynı 
#kalacağını (sabit hızlı hareket varsayımı - Constant Velocity Model) belirtir.
#==============================================

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
        # Kalman'ın kendi tahmin ettiği 8 boyutlu dünyası ile YOLO'nun verdiği 4 boyutlu gerçek dünyayı eşleştirdiği köprüdür.
        # Bu matris, 8 durumlu vektörü alır ve sadece ilk 4 elemanı ($x, y, w, h$) "1" ile çarparak alır, hızları ($vx, vy, vw, vh$) 
        # ise "0" ile çarparak yok sayar. Çünkü YOLO bize hız veremez, hızı Kalman kendisi zaman içindeki konum değişimlerinden türetecektir.
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], np.float32)

        # Process Noise Covariance (Q)
        # Adjust these to trust model prediction (low noise) or allow flexibility
        # Matematiksel bir otopilot için en kritik ayarlardan biri! Bu değer, "Kendi tahmin ettiğim fiziksel modele (Sabit Hız Modeli) ne kadar güvenmeliyim?" sorusunun cevabıdır.
        # np.eye(8): Çaprazında 1'ler olan 8x8 bir birim matris oluşturur.
        # * 0.03: Bu değer oldukça düşüktür. Yani Kalman'a şunu diyoruz: "Savaşan İHA hedefini takip ederken, hedefin havada aniden teleport olmayacağını (fizik kurallarına uyacağını) 
        # biliyorum. O yüzden kendi hız tabanlı tahmin modeline GÜVEN. Dış etkenler (rüzgar vb.) yüzünden olacak sapmalar azdır (0.03)." Eğer bu değeri büyütseydin (örn: 0.5), 
        # filtre tahmine değil, gelen ölçüme daha çok güvenmeye başlardı.
        self.kf.processNoiseCov = np.eye(8, dtype=np.float32) * 0.03

        # Measurement Noise Covariance (R)
        # Trust measurements fairly well
        # Bu da "Sensörümden (YOLO) gelen veriye ne kadar güvenmeliyim?" sorusunun cevabıdır.
        # * 0.1: Bu değer, process noise (0.03) değerinden daha büyüktür. Bu demektir ki: "YOLO bazen kutuyu titretebilir, 
        # pervaneden dolayı hedefi biraz sağda veya solda algılayabilir. Yani sensörümde bir miktar 'titreme/gürültü' var. 
        # YOLO'nun dediği konuma %100 inanma, kendi tahmininle (Q) harmanla ve bana daha yumuşak, pürüzsüz (smooth) bir rota ver."
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.1 # 1e-1

        # Error Covariance (P)
        # Sistemin başlangıçtaki "Belirsizlik" veya "Şüphe" seviyesidir. Başlangıçta (1.0 gibi yüksek bir değer), 
        # Kalman hedefin hızı hakkında hiçbir şey bilmediği için tahmine değil, doğrudan ilk gelen YOLO verisine inanır. 
        # Birkaç frame geçtikten sonra filtre hızı öğrendikçe bu $P$ matrisi kendi kendine küçülür (şüphe azalır) ve filtre oturur.Bu __init__ kısmı, 
        # sistemin "DNA'sının" yazıldığı yerdir. 
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
        Doğru Kalman döngüsü: predict() → correct() sırasıyla çağrılmalıdır.
        predict() hız vektörlerini (vx, vy) günceller, correct() ise kameradan
        gelen gerçek ölçümle tahmini düzeltir.
        """
        if not self.active:
            self.init(box)
            return box

        # 1) ÖNCELİKLE TAHMİN YAP — Bu adım hız vektörlerini (vx, vy, vw, vh) günceller.
        #    Bu yapılmazsa filtre hedef kaybolduğunda doğru tahmin yapamaz.
        self.kf.predict()

        # 2) SONRA GERÇEK ÖLÇÜMLE DÜZELT — Kameradan gelen kutu koordinatlarıyla
        #    tahmini birleştirerek en iyi sonuca ulaşır.
        measurement = np.array([[box[0]], [box[1]], [box[2]], [box[3]]], dtype=np.float32)
        self.kf.correct(measurement)

        # Düzeltilmiş (corrected) durumu al
        corrected = self.kf.statePost
        self.active = True
        self.missed_frames = 0

        # corrected (8, 1) matristir — sadece ilk 4 değer [x, y, w, h] bizi ilgilendirir
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
