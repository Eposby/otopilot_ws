#!/usr/bin/env python3
"""
YOLO Detector — Nesne Algılama Modülü
=======================================

YOLO modelini yükler, frame üzerinde nesne tespiti yapar ve
Detection nesneleri döndürür. Vision Processor tarafından kullanılır.

Kullanım:
    detector = YOLODetector(model_path="yolov8n.pt", target_classes=["airplane"])
    detector.load_model()
    detections = detector.detect(frame)
    primary = detector.get_primary_target(detections)
    error_x, error_y = detector.calculate_error(primary, frame_center)

Tip Notları:
    Optional[X]   → X tipi veya None olabilir    | Optional[str] = str veya None
    Tuple[X, Y]   → (X, Y) formatında tuple      | Tuple[int, int] = (320, 240)



Neden Node Değil?
YOLO tek başına bir şey yapmaz. Ona bir görüntü verirsin, sana bir tespit döndürür. Kendi başına kamera dinlemez, topic yayınlamaz. O bir alettir, bir işçi değil.

Eğer node yapsaydık ne olurdu?

yolo_detector_node kameradan image alır → Detection yayınlar.
vision_processor_node bu Detection'ı dinler → TargetInfo'ya çevirir → Beyne gönderir.
Sorun: Gereksiz gecikme! Veri iki node arasında ROS topic üzerinden atlayacak (serialization + deserialization). Görüntü işlemede her milisaniye kritik.


Şu anki yapıda ne oluyor?

vision_processor kameradan image alır → aynı fonksiyon içinde yolo_detector.detect(frame) çağırır → Sonucu hemen TargetInfo'ya çevirir.
Sıfır ek gecikme. Veri hiç ROS kanalına çıkmadan işleniyor.


❌ Yanlış: Kamera → [YOLO Node] → topic → [Vision Node] → topic → Beyin
                         ↑ gereksiz gecikme

✅ Doğru:  Kamera → [Vision Node (YOLO içinde)] → topic → Beyin
                         ↑ tek adım, hızlı


yolo_detector.py
 bir kütüphane dosyasıdır, tıpkı Python'daki math veya numpy gibi. Onu import edip kullanırsın ama tek başına çalıştırmazsın. Bu yaklaşım hem daha hızlı hem de daha temizdir.
"""

import cv2
import numpy as np
from typing import Optional, List, Tuple
from dataclasses import dataclass


@dataclass
class Detection:
    """Tespit edilen nesne bilgisi"""
    class_id: int
    class_name: str
    confidence: float       # %50'den büyük ise geçerli
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center: Tuple[int, int]  # center_x, center_y
    area: int


class YOLODetector:
    """YOLO nesne algılama sınıfı"""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
        target_classes: Optional[List[str]] = None,
        device: str = "auto"
    ):
        """
        Args:
            model_path: YOLO model dosya yolu
            confidence_threshold: Minimum güven eşiği
            target_classes: Takip edilecek sınıf isimleri (varsayılan: ["airplane"])
            device: 'cpu', 'cuda', veya 'auto'
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.target_classes = target_classes or ["airplane"]
        self.device = device
        self.model = None
        self.class_names = {}

    def load_model(self) -> bool:
        """YOLO modelini yükle"""
        try:
            from ultralytics import YOLO
            # Neden burada import? Eğer kütüphane yoksa hata yakalarız.
            print(f"[YOLO] Model yükleniyor: {self.model_path}")
            self.model = YOLO(self.model_path)

            # Cihaz seçimi
            if self.device == "auto":
                import torch
                self.device = "cuda" if torch.cuda.is_available() else "cpu"

            # Sınıf isimlerini al
            self.class_names = self.model.names
            print(f"[YOLO] Model yüklendi. Cihaz: {self.device}")
            print(f"[YOLO] Sınıflar: {list(self.class_names.values())[:10]}...")
            print(f"[YOLO] Hedef sınıflar: {self.target_classes}")

            return True

        except ImportError:
            print("[HATA] ultralytics kütüphanesi bulunamadı!")
            print("Kurulum: pip install ultralytics")
            return False
        except Exception as e:
            print(f"[HATA] Model yüklenemedi: {e}")
            return False

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Frame üzerinde nesne tespiti yap

        Args:
            frame: BGR formatında görüntü (OpenCV)

        Returns:
            Detection listesi (alana göre büyükten küçüğe sıralı)
        """
        if self.model is None:
            return []

        detections = []

        results = self.model(
            frame,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False
        )

        for result in results:
            boxes = result.boxes

            if boxes is None:
                continue

            for box in boxes:
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Sınıf ve güven
                class_id = int(box.cls[0])
                class_name = self.class_names.get(class_id, "unknown")
                confidence = float(box.conf[0])

                # Hedef sınıf filtresi
                if self.target_classes and class_name not in self.target_classes:
                    continue  # İstemediğimiz sınıfı atla

                # Merkez hesapla
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Alan hesapla
                area = (x2 - x1) * (y2 - y1)

                detection = Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(x1, y1, x2, y2),
                    center=(center_x, center_y),
                    area=area
                )
                detections.append(detection)

        # Alana göre sırala (en büyük önce)
        detections.sort(key=lambda d: d.area, reverse=True)

        return detections

    def detect_and_draw(
        self,
        frame: np.ndarray,
        draw_center: bool = True,
        draw_info: bool = True
    ) -> Tuple[np.ndarray, List[Detection]]:
        """
        Tespit yap ve görselleştir

        Returns:
            (annotated_frame, detections)
        """
        detections = self.detect(frame)
        annotated = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx, cy = det.center

            # Bounding box
            color = (0, 255, 0)  # Yeşil
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Merkez noktası
            if draw_center:
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(annotated, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 2)
                cv2.line(annotated, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 2)

            # Bilgi yazısı
            if draw_info:
                label = f"{det.class_name}: {det.confidence:.2f}"
                cv2.putText(
                    annotated, label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2
                )

        return annotated, detections

    # bu kısım değişecek çünkü sadece alanın büyüklüğüne bakarak öncelik vermeyeceğiz
    # gerekirse en uzaktaki cisim vurulacak
    def get_primary_target(
        self,
        detections: List[Detection]
    ) -> Optional[Detection]:
        """
        En büyük/birincil hedefi döndür
        """
        if not detections:
            return None
        return detections[0]  # Zaten alana göre sıralı

    def calculate_error(
        self,
        detection: Detection,
        frame_center: Tuple[int, int]
    ) -> Tuple[int, int]:
        """
        Hedefin frame merkezinden sapmasını hesapla.
        Bu değerler ileride PID kontrolcüsüne girdi olarak verilecek.

        Args:
            detection: Tespit edilen nesne
            frame_center: Frame merkezi (x, y)

        Returns:
            (error_x, error_y) piksel cinsinden hata
            error_x > 0 → hedef sağda
            error_x < 0 → hedef solda
            error_y > 0 → hedef aşağıda
            error_y < 0 → hedef yukarıda
        """
        error_x = detection.center[0] - frame_center[0]
        error_y = detection.center[1] - frame_center[1]
        return error_x, error_y







# 📦 @dataclass - Veri Sınıfı Dekoratörü
# dataclass, sadece veri tutan sınıfları hızlıca oluşturmak için kullanılır.
#============================================================================

# # ❌ NORMAL SINIF (uzun, tekrarlı)
# class Detection:
#     def __init__(self, class_id, class_name, confidence, bbox, center, area):
#         self.class_id = class_id
#         self.class_name = class_name
#         self.confidence = confidence
#         self.bbox = bbox
#         self.center = center
#         self.area = area
    
#     def __repr__(self):
#         return f"Detection(class_id={self.class_id}, ...)"
    
#     def __eq__(self, other):
#         return (self.class_id == other.class_id and 
#                 self.class_name == other.class_name and ...)


# # ✅ DATACLASS (kısa, otomatik)
# from dataclasses import dataclass

# @dataclass
# class Detection:
#     class_id: int
#     class_name: str
#     confidence: float
#     bbox: tuple
#     center: tuple
#     area: int




# Metod	Ne Yapar	Örnek
# init  Constructor 	Detection(0, "person", 0.95, ...)
# __repr__	Yazdırma	print(det) → güzel çıktı
# __eq__	Karşılaştırma	det1 == det2



# detection/yolo_detector.py
# @dataclass
# class Detection:
#     class_id: int
#     class_name: str
#     confidence: float
#     bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
#     center: Tuple[int, int]
#     area: int

# # Kullanım:
# det = Detection(
#     class_id=0,
#     class_name="person",
#     confidence=0.92,
#     bbox=(100, 150, 300, 400),
#     center=(200, 275),
#     area=37500
# )

# print(det.class_name)  # "person"
# print(det.confidence)  # 0.92
# print(det.bbox)        # (100, 150, 300, 400)
# print(det.center)      # (200, 275)
# print(det.area)        # 37500


# @dataclass
# class MotorCommand:
#     pan_angle: float = 0.0      # Varsayılan değer
#     tilt_angle: float = 0.0
#     pan_speed: float = 100.0
#     tilt_speed: float = 100.0

# # Varsayılan değerlerle oluştur:
# cmd = MotorCommand()  # Tüm değerler varsayılan
# cmd = MotorCommand(pan_angle=45.0)  # Sadece birini değiştir
#============================================================================






#============================================================================

# torch.cuda.is_available()
#         │
#         ├─ True  → NVIDIA GPU var → "cuda" kullan (10x hızlı!)
#         │
#         └─ False → GPU yok → "cpu" kullan



#         # 4. Model içindeki sınıf isimlerini al
#         self.class_names = self.model.names
#         # Örnek: {0: 'person', 1: 'bicycle', 2: 'car', ...}

#============================================================================







#YOLO içi
#============================================================================



# # ultralytics kütüphanesinde (biz yazmadık, hazır geliyor)
# class YOLO:
#     def __call__(self, source, conf=0.25, device="auto", verbose=True, ...):
#         """Model çağrıldığında çalışır"""
#         # source = görüntü
#         # conf = güven eşiği
#         # device = cpu/cuda
#         # verbose = log yaz/yazma
        
#         # ... nesne tespiti yap ...
#         return results






# self.model = YOLO("yolov8n.pt")

# # Bu ikisi AYNI şeyi yapar:
# results = self.model(frame)                    # Varsayılan parametreler
# results = self.model.__call__(frame)           # Aynı şey

# # Ekstra parametrelerle:
# results = self.model(frame, conf=0.5, device="cuda")




#self.model yolo içinde __call__ metodunu çağırıyor bunun içinde birçok parametre var
#bu parametreleri yolo_detector.py içinde tanımlıyoruz
#self.model = YOLO("yolov8n.pt")
# burada şunlar var 
# source = görüntü
# conf = güven eşiği
# device = cpu/cuda
# verbose = log yaz/yazma
# box = kutu
# iou = 
# 



# detect
#  fonksiyonu her çağrıldığında sadece kendisine verilen o anki tek bir frame üzerinde çalışır.

# results = self.model(frame): Bu satır, YOLO modelini o anki fotoğraf karesi (frame) üzerinde çalıştırır.
# for result in results: YOLO bazen (video modunda) birden çok sonuç paketi döndürebilir ama burada tek bir frame verdiğimiz için genellikle tek bir sonuç paketi döner.
# boxes = result.boxes: İşte burası! boxes değişkeni, o anki frame içinde bulunan tüm nesnelerin listesidir. Eğer o karede 3 insan, 1 araba varsa, boxes listesinin uzunluğu 4 olur.
# for box in boxes: Bu döngü de o karede bulunan her bir nesneyi tek tek işler (koordinatını alır, sınıfına bakar, listeye ekler).
# Yani döngü her frame için baştan çalışır. detections listesi her frame için sıfırdan oluşturulur (detections = []).

# Özetle akış şöyledir: Kamera yeni frame gönderir -> 
# detect(frame)
#  çalışır -> O karedeki tüm nesneler bulunur (boxes) -> Hepsi detections listesine doldurulup geri gönderilir. Sonraki frame gelince her şey baştan başlar.



# # Ultralytics kütüphanesinde (hazır geliyor):
# class Results:
#     def __init__(self):
#         self.boxes = Boxes(...)      # ← Kutu bilgileri
#         self.masks = Masks(...)      # Segmentasyon maskeleri
#         self.probs = Probs(...)      # Sınıflandırma olasılıkları
#         self.keypoints = ...         # Pose keypoints



# results = self.model(frame)  # Liste döner: [Result1, Result2, ...]

# for result in results:
#     boxes = result.boxes     # result nesnesinin boxes özelliği
#     #       ↑      ↑
#     #    nesne  özellik (attribute)




# result
#   │
#   ├── boxes ─────┬── xyxy  → koordinatlar
#   │              ├── cls   → sınıf ID
#   │              └── conf  → güven
#   │
#   ├── masks (segmentasyon için)
#   └── probs (sınıflandırma için)
#============================================================================






#detect içindeki box.xyxy[0].cpu().numpy().astype(int)
#============================================================================
#x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)


#box.xyxy
# tensor([[102.5, 148.3, 305.7, 412.9]], device='cuda:0')
# ↑ GPU'da, 2D tensor, float değerler


#box.xyxy[0]
# tensor([102.5, 148.3, 305.7, 412.9], device='cuda:0')
# ↑ [0] ile ilk satırı al → 1D tensor



#box.xyxy[0].cpu()
# tensor([102.5, 148.3, 305.7, 412.9])
# ↑ GPU'dan CPU'ya taşı (numpy için gerekli)



#box.xyxy[0].cpu().numpy()
# array([102.5, 148.3, 305.7, 412.9], dtype=float32)
# ↑ PyTorch tensor → NumPy array




#box.xyxy[0].cpu().numpy().astype(int)
# array([102, 148, 305, 412])
# ↑ Float → Integer (piksel koordinatları tam sayı olmalı)



#x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
# x1=102, y1=148, x2=305, y2=412
# ↑ Unpacking: 4 elemanlı array → 4 değişken





# box.xyxy[0]           .cpu()              .numpy()            .astype(int)
#      │                   │                    │                     │
#      ▼                   ▼                    ▼                     ▼
# [102.5, 148.3,    →  [102.5, 148.3,   →  [102.5, 148.3,    →  [102, 148,
#  305.7, 412.9]        305.7, 412.9]       305.7, 412.9]        305, 412]
#      │                   │                    │                     │
#   GPU Tensor         CPU Tensor          NumPy Array          Int Array
#============================================================================







# #fonksiyon çalışma mantığı 
# #============================================================================
# # main.py içinde
# while self.running:
#     # 1. Kameradan frame al
#     ret, frame = self.camera.read()
    
#     # 2. Her frame için detect_and_draw çağrılır
#     annotated, detections = self.detector.detect_and_draw(frame)
#     #                                                      ↑
#     #                                            Yeni frame, yeni tespit
    
#     # 3. Ekranda göster
#     cv2.imshow("Tracker", annotated)



# Zaman   Frame    detect_and_draw()     Sonuç
# ──────────────────────────────────────────────────
# t=0     Frame1   → YOLO çalışır      → [person at (200,300)]
# t=33ms  Frame2   → YOLO çalışır      → [person at (210,305)]  ← Hareket etti
# t=66ms  Frame3   → YOLO çalışır      → [person at (220,310)]
# t=99ms  Frame4   → YOLO çalışır      → []  ← Hedef kayboldu
# ...



# def detect_and_draw(self, frame):
#     # 1. Tespit yap
#     detections = self.detect(frame)  # ← Her seferinde yeni frame analiz edilir
    
#     annotated = frame.copy()  # ← Orijinali bozmamak için kopya
    
#     # 2. Her tespit için çiz
#     for det in detections:
#         cv2.rectangle(...)   # Kutu çiz
#         cv2.circle(...)      # Merkez çiz
#         cv2.putText(...)     # Yazı yaz
    
#     # 3. İşlenmiş frame + tespitler döndür
#     return annotated, detections





# frame (orijinal)      annotated (kopya)
# ┌────────────┐       ┌────────────┐
# │            │       │ ┏━━━━┓     │
# │            │  ───> │ ┃ 😊 ┃     │  ← Kutular çizildi
# │            │       │ ┗━━━━┛     │
# └────────────┘       └────────────┘
#      ↓                    ↓
#    Temiz            Görselleştirilmiş

#============================================================================