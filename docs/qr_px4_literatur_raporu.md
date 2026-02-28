# Teknofest Savaşan İHA - QR Kod Okuma ve PX4 Literatür Araştırması

## 📋 İçindekiler
1. [Senaryonuzun Analizi](#senaryonuzun-analizi)
2. [QR Kod Okuma Optimizasyonu](#qr-kod-okuma-optimizasyonu)
3. [PX4/MAVROS Yazılımı](#px4mavros-yazılımı)
4. [Önerilen Mimari ve Algoritma](#önerilen-mimari-ve-algoritma)
5. [Kaynaklar](#kaynaklar)

---

## 🎯 Senaryonuzun Analizi

### Mevcut Durum
- **QR Kod Boyutu**: 2m × 2m
- **Koruyucu Plakalar**: 45° açılı, 3m yüksekliğinde (düz uçuşta okumayı engeller)
- **Minimum Pull-up İrtifası**: 50m
- **Mevcut Yaklaşım** (`qr2.py`): Kamikaze dalış manevrası ile 45° açıyla QR'a yaklaşım

### Geometrik Analiz

```
       Uçak (50m irtifa)
           ↓ (45° dalış)
           \
            \  ← Görüş açısı
             \
    ┌─────────┐ ← 3m plaka
    │   QR    │ ← 2m × 2m kod
    └─────────┘
```

**Kritik Hesaplama:**
- 50m irtifadan 45° dalış açısıyla bakıldığında, QR kodun görülebilmesi için plakaların iç tarafından bakılması gerekir
- Plakaların 45° açısı, uçağın dalış açısıyla uyumlu → **dalış sırasında QR görülebilir**

---

## 📱 QR Kod Okuma Optimizasyonu

### 1. Temel Zorluklar

| Zorluk | Açıklama | Çözüm Yaklaşımı |
|--------|----------|-----------------|
| **Küçük Görüntü Boyutu** | 50m'den 2m kod çok küçük görünür | Yüksek çözünürlüklü kamera, zoom |
| **Hareket Bulanıklığı** | Yüksek hızlı dalışta blur | Kısa pozlama, görüntü stabilizasyonu |
| **Perspektif Bozulması** | 45° açıdan bakış | Perspektif düzeltme algoritması |
| **Işık Koşulları** | Değişken aydınlatma | Adaptif threshold, kontrast artırma |

### 2. Önerilen Görüntü İşleme Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    QR KOD OKUMA PİPELİNE                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   📷 Kamera Girişi (Yüksek FPS, Düşük Pozlama)                 │
│           ↓                                                     │
│   🔲 Ön İşleme                                                  │
│   ├── Grayscale dönüşüm                                         │
│   ├── Gaussian blur (gürültü azaltma)                           │
│   ├── CLAHE (Contrast Limited Adaptive Histogram Equalization)  │
│   └── Adaptive thresholding                                     │
│           ↓                                                     │
│   🎯 QR Tespiti (İki Aşamalı)                                   │
│   ├── Aşama 1: YOLO-based QR lokalizasyonu (hız için)          │
│   └── Aşama 2: ROI extraction + Pyzbar decode (doğruluk için)  │
│           ↓                                                     │
│   📐 Perspektif Düzeltme                                        │
│   ├── Corner detection (QR finder patterns)                     │
│   ├── Homography matrix hesaplama                               │
│   └── Perspective warp transform                                │
│           ↓                                                     │
│   ✅ Decode & Doğrulama                                         │
│   ├── Pyzbar ile decode                                         │
│   ├── Checksum doğrulama                                        │
│   └── Birden fazla frame'de tutarlılık kontrolü                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Kamera Gereksinimleri

> [!IMPORTANT]
> **Minimum Piksel Sayısı Hesabı:**
> - 50m mesafeden 2m kod → Görüş açısına bağlı piksel boyutu
> - QR kodun başarıyla okunması için minimum **33 modül × 10 piksel/modül** gerekli
> - **Tavsiye**: En az 21 versiyon QR için ~330 piksel genişlik hedeflenmelidir

| Parametre | Önerilen Değer |
|-----------|---------------|
| Çözünürlük | 1920×1080 (minimum) |
| FPS | 60+ fps |
| Pozlama | 1/1000s veya daha kısa |
| Lens | 35-50mm eşdeğeri (dar FOV) |
| Gimbal | 2-eksen stabilizasyon önerilir |

### 4. Algoritma Seçenekleri

#### Seçenek A: Geleneksel OpenCV + Pyzbar
```python
# Basit ve güvenilir yaklaşım
import cv2
from pyzbar.pyzbar import decode

def detect_qr(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Kontrastı artır
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    
    # Adaptif threshold
    binary = cv2.adaptiveThreshold(
        enhanced, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY, 11, 2
    )
    
    # Decode
    codes = decode(binary)
    return codes
```

#### Seçenek B: Deep Learning + Pyzbar (Önerilen)
```python
# YOLOv8 ile QR lokalizasyonu + Pyzbar decode
from ultralytics import YOLO
from pyzbar.pyzbar import decode

class QRDetector:
    def __init__(self):
        self.yolo = YOLO("yolov8n-qr.pt")  # QR için eğitilmiş model
        
    def detect(self, frame):
        # Önce YOLO ile QR bölgesini bul
        results = self.yolo(frame, conf=0.5)
        
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            roi = frame[y1:y2, x1:x2]
            
            # ROI üzerinde perspektif düzeltme ve decode
            corrected = self.perspective_correction(roi)
            codes = decode(corrected)
            
            if codes:
                return codes[0].data.decode()
        return None
```

#### Seçenek C: Çoklu Frame Fusion (En Güvenilir)
```python
class MultiFrameQRDecoder:
    def __init__(self, min_confirmations=3):
        self.buffer = []
        self.min_confirmations = min_confirmations
        
    def process(self, frame):
        result = detect_qr(frame)
        if result:
            self.buffer.append(result)
            
        # En az N kez aynı sonuç
        if len(self.buffer) >= self.min_confirmations:
            if len(set(self.buffer[-self.min_confirmations:])) == 1:
                return self.buffer[-1]
        return None
```

### 5. Dalış Sırasında Lateral Düzeltme (Mevcut Kodunuz)

Kodunuzdaki `execute_kamikaze()` fonksiyonunda lateral düzeltme zaten mevcut:

```python
# Cross-track error ile lateral düzeltme
lateral_error = dx * math.sin(yaw_rad) - dy * math.cos(yaw_rad)
correction_roll = -ROLL_GAIN * lateral_error
```

> [!TIP]
> **Geliştirme Önerisi:**
> Visual servoing ekleyerek QR kodun görüntü merkezinde tutulması sağlanabilir:
> ```python
> # Kamera merkezinden QR offset hesapla
> qr_center_x = detected_qr.rect.left + detected_qr.rect.width / 2
> image_center_x = frame.shape[1] / 2
> pixel_error = qr_center_x - image_center_x
> 
> # Roll düzeltmesi (piksel hatası → roll açısı)
> VISUAL_GAIN = 0.01  # derece/piksel
> visual_roll_correction = VISUAL_GAIN * pixel_error
> ```

---

## 🛩️ PX4/MAVROS Yazılımı

### 1. PX4 Kaynak Kod Yapısı

```
PX4-Autopilot/
├── src/
│   ├── modules/
│   │   ├── fw_att_control/    ← Sabit kanat attitude kontrolü
│   │   ├── fw_pos_control/    ← Sabit kanat pozisyon kontrolü
│   │   ├── navigator/         ← Görev yönetimi
│   │   └── commander/         ← Mod yönetimi
│   ├── lib/
│   │   └── tecs/              ← Total Energy Control System
│   └── drivers/
├── platforms/
└── boards/
```

### 2. PX4 Yazılımını Değiştirmek

> [!IMPORTANT]
> **Evet, PX4 kaynak kodu değiştirilebilir!** Açık kaynak kodludur ve GitHub'da barındırılır.

#### Yöntem 1: Parametre Ayarlama (En Kolay)
Çoğu davranış parametre değişikliği ile düzeltilebilir - kod değişikliği gerekmez:

| Parametre | Açıklama | Kamikaze İçin Değer |
|-----------|----------|---------------------|
| `FW_P_LIM_MIN` | Minimum pitch limiti | -85° |
| `FW_T_SINK_MAX` | Maksimum batma hızı | 15 m/s |
| `FW_AIRSPD_MAX` | Maksimum hava hızı | 35 m/s |
| `FW_T_SPDWEIGHT` | TECS hız ağırlığı | 0.0 (dalışta) |

#### Yöntem 2: ROS2 External Mode (Orta Seviye)
MAVROS üzerinden kontrol - firmware değişikliği gerektirmez:

```python
# Attitude control (mevcut yaklaşımınız)
self.publish_attitude(roll=0.0, pitch=45.0, yaw=target_yaw, thrust=0.1)
```

#### Yöntem 3: Kaynak Kod Modifikasyonu (İleri Seviye)

```bash
# PX4 kaynak kodunu klonla
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Geliştirme ortamı kurulumu
bash ./Tools/setup/ubuntu.sh

# Örnek: fw_att_control modülünü düzenle
code src/modules/fw_att_control/

# Derleme (SITL için)
make px4_sitl gazebo

# Holybro/Pixhawk için derleme
make px4_fmu-v5_default
```

##### Özel Uçuş Modu Ekleme Adımları:

1. **Yeni modül oluştur**: `src/modules/custom_kamikaze/`
2. **CMakeLists.txt ekle**
3. **uORB mesajları tanımla** (gerekirse)
4. **Commander'a mod kaydı yap**
5. **Derle ve test et**

> [!WARNING]
> **Kaynak Kod Değişikliği Riskleri:**
> - PX4 güncellemelerinde merge conflict'ler
> - Güvenlik sertifikası geçersizleşebilir
> - Beklenmeyen davranışlar
> - Test/debugging zorluğu

### 3. MAVROS ile Sabit Kanat Kontrol

> [!CAUTION]
> PX4'ün "Offboard" modu sabit kanat uçaklar için **resmi olarak desteklenmez** (multirotor ve VTOL içindir). Ancak pratikte attitude control çalışır!

#### Mevcut Kodunuzdaki Yaklaşım (Doğru):
```python
def publish_attitude(self, roll, pitch, yaw, thrust):
    msg = AttitudeTarget()
    msg.type_mask = 7  # IGNORE rates - sadece attitude kullan
    msg.orientation = euler_to_quaternion(roll, pitch, yaw)
    msg.thrust = thrust
    self.attitude_pub.publish(msg)
```

#### Alternatif: SetpointGlobal (Waypoint Bazlı)
```python
from mavros_msgs.msg import GlobalPositionTarget

def send_dive_waypoint(self, lat, lon, alt):
    msg = GlobalPositionTarget()
    msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    msg.type_mask = 0b0000111111111000  # Sadece position
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    self.global_setpoint_pub.publish(msg)
```

### 4. Kamikaze Dalış İçin Kritik PX4 Parametreleri

```yaml
# QGroundControl'de ayarlanacak parametreler
FW_P_LIM_MIN: -85        # Dalış açısı izni
FW_P_LIM_MAX: 45         # Pull-up için
FW_T_SINK_MAX: 15        # Hızlı iniş
FW_T_SINK_R_SP: 2.0      # Batma hızı setpoint
FW_AIRSPD_MAX: 35        # Dalışta yüksek hız
FW_AIRSPD_TRIM: 20       # Normal cruise
FW_THR_IDLE: 0.05        # Dalışta düşük throttle
```

---

## 🏗️ Önerilen Mimari ve Algoritma

### Genel Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────────┐
│                      COMPANION COMPUTER                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │ Kamera Node     │  │ QR Detector     │  │ State Machine   │ │
│  │ (capture.py)    │→ │ (qr_detector.py)│→ │ (qr2.py)        │ │
│  │                 │  │                 │  │                 │ │
│  │ 60fps, 1080p    │  │ YOLO + Pyzbar   │  │ MAVROS commands │ │
│  └─────────────────┘  └─────────────────┘  └────────┬────────┘ │
│                                                      │          │
│                                              MAVROS  │          │
│                                                      ↓          │
├─────────────────────────────────────────────────────────────────┤
│                       FLIGHT CONTROLLER                          │
│                     (PX4 + Pixhawk)                              │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  OFFBOARD Mode → Attitude Controller → Control Surfaces  │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Geliştirilmiş State Machine (qr2.py için öneri)

```python
class QRMissionStates(Enum):
    TAKEOFF = 1
    CRUISE_TO_QR = 2
    SLOWDOWN = 3
    QR_DETECTION_PHASE = 4   # YENİ: Aktif QR arama
    KAMIKAZE_DIVE = 5
    QR_READING = 6           # YENİ: Dalış sırasında okuma
    PULLUP = 7
    RECOVERY = 8
    RTL = 9
    LAND = 10

# Dalış sırasında QR okuma entegrasyonu
def execute_kamikaze(self):
    # ... mevcut kod ...
    
    # QR okuma callback'i kontrol et
    if self.qr_detected and self.qr_data:
        self.get_logger().info(f"✓ QR OKUNDU: {self.qr_data}")
        # Görev tamamlandı, pull-up'a geç
        self.task_state = 2  # PULL-UP
```

### Önerilen Dalış Profili

```
Altitude (m)
    ↑
110 ├────────────────┐
    │ CRUISE        └────────┐ SLOWDOWN (300m → 110m)
100 ├                        └────────┐
    │                                  \ KAMIKAZE (45°)
 80 ├                                   \
    │                                    \  ← QR okunur (bu aralıkta)
 60 ├                                     \
    │                                      \
 50 ├──────────────────────────────────────┼─ PULL-UP başlar
    │                                      ↗
 70 ├                                     /  PULL-UP aşama 1
    │                                    /
 85 ├                                   /   PULL-UP aşama 2
    │                                  /
    └──────────────────────────────────────→ Mesafe
                                    QR
```

---

## 📚 Kaynaklar

### Akademik Makaleler

1. **YOLOv7-UAV for Small Object Detection** - MDPI (2024)
   - UAV'dan küçük nesne tespiti için optimize edilmiş YOLO
   - [mdpi.com](https://www.mdpi.com)

2. **SOD-YOLO: YOLOv8 Based Small Object Detection** - MDPI (2024)
   - Multiscale feature fusion ile küçük nesne tespiti

3. **QR Code Detection and Decoding from Drones** - scitepress.org (2024)
   - YOLO + Pyzbar hibrit yaklaşım

4. **Image-Based Visual Servoing for UAV Precision Landing** - ResearchGate
   - QR kod tabanlı görsel servoing

### Teknik Dokümantasyon

1. **PX4 Developer Guide**: [docs.px4.io](https://docs.px4.io)
2. **PX4 Fixed-Wing Controller Tuning**: [docs.px4.io/main/en/config_fw/](https://docs.px4.io/main/en/config_fw/)
3. **MAVROS Wiki**: [wiki.ros.org/mavros](http://wiki.ros.org/mavros)
4. **Pyzbar Documentation**: [github.com/NaturalHistoryMuseum/pyzbar](https://github.com/NaturalHistoryMuseum/pyzbar)

### GitHub Repositories

1. **PX4-Autopilot**: `https://github.com/PX4/PX4-Autopilot`
2. **MAVROS**: `https://github.com/mavlink/mavros`
3. **YOLOv8**: `https://github.com/ultralytics/ultralytics`
4. **OpenCV QR Examples**: `https://github.com/opencv/opencv/tree/master/samples/python`

### Video Kaynakları

1. **PX4 Custom Flight Modes with ROS2** - YouTube
2. **Fixed Wing Tuning Guide** - PX4 YouTube Channel
3. **UAV QR Code Detection Systems** - IEEE Conference Videos

---

## 🔧 Sonraki Adımlar (Öneriler)

1. **Kamera Seçimi ve Testi**
   - 50m'den 2m×2m QR'ı deneysel olarak test edin
   - Farklı lens/çözünürlük kombinasyonları deneyin

2. **QR Detector Modülü Geliştirme**
   - Ayrı bir ROS node olarak QR okuyucu yazın
   - `qr2.py` ile subscription üzerinden entegre edin

3. **Simülasyon Ortamı**
   - Gazebo'ya sanal QR kod ekleyin
   - Dalış sırasında okuma testleri yapın

4. **PX4 Parametre Tuning**
   - TECS parametrelerini optimize edin
   - Dalış/pull-up geçişlerini test edin

5. **Failsafe Mekanizmaları**
   - QR okunamazsa alternatif davranış
   - Pull-up başarısız olursa RTL
