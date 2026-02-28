# Sabit Kanatlı İHA ile Hava-Hava Hedef Takibi

## Literatür Taraması ve Uygulama Rehberi

Bu doküman, sabit kanatlı bir İHA ile başka bir uçağı 4 saniye boyunca kesintisiz takip etmek için gerekli bilgileri içerir.

---

## BÖLÜM 1: LİTERATÜR TARAMASI

### 1. Genel Mimari - Sistem Bileşenleri

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    HEDEF TAKİP SİSTEMİ MİMARİSİ                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌──────────────┐      ┌──────────────┐      ┌──────────────┐         │
│   │   ALGILAMA   │ ───► │    TAKİP     │ ───► │   GÜDÜM      │         │
│   │  (Perception)│      │  (Tracking)  │      │  (Guidance)  │         │
│   └──────────────┘      └──────────────┘      └──────────────┘         │
│         │                      │                      │                 │
│         ▼                      ▼                      ▼                 │
│   • YOLO Object Det.     • Kalman Filter      • Proportional Nav.      │
│   • Camera + Gimbal      • DeepSORT           • Pure Pursuit           │
│   • Thermal/Optical      • KCF Tracker        • Visual Servoing        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

### 2. Aşamalar ve Yöntemler

#### 📷 AŞAMA 1: Hedef Algılama (Detection)

| Yöntem | Açıklama | Kaynak |
|--------|----------|--------|
| **YOLO (v5/v8/v11)** | Gerçek zamanlı nesne tespiti, tek geçişte sınıf + bbox | arxiv.org |
| **Air-YOLO, AE-YOLO** | Hava görüntüleri için özelleştirilmiş YOLO | IEEE |
| **Gimbal Kamera** | 3-eksen stabilizasyon, ±0.01° hassasiyet | mdpi.com |
| **Thermal + Optical** | Çift sensör füzyonu | Commercial Gimbals |

**Önemli:** Küçük hedef tespiti (small object detection) ciddi bir zorluk. Çözüm: Adaptive scale fusion, feature pyramid networks.

---

#### 🎯 AŞAMA 2: Hedef Takibi (Tracking)

Hedef algılandıktan sonra frame'ler arası takip:

| Algoritma | Avantaj | Dezavantaj |
|-----------|---------|------------|
| **Kalman Filter** | Basit, hızlı, pozisyon tahmini | Doğrusal hareket varsayımı |
| **DeepSORT** | Deep learning + appearance | GPU gerektirir |
| **KCF (Kernelized CF)** | Hızlı, az kaynak | Ölçek değişimine zayıf |
| **MOSSE** | Çok hızlı | Basit hedefler için |

**Önerilen:** Kalman Filter + YOLO kombinasyonu

---

#### ✈️ AŞAMA 3: Güdüm Algoritması (Guidance Law)

Bu en kritik aşama - kamerada hedefi 4 saniye boyunca tutmak için:

##### A) Proportional Navigation (PN) - Oransal Yönlendirme
```
Komut Açı Hızı = N × (Görüş Hattı Açı Hızı)
```
- **N** = Navigasyon kazancı (genellikle 3-5)
- Füze güdümünde altın standart
- Hedefin görüş hattını sabit tutmayı hedefler

**Kaynak:** Wikipedia - Proportional Navigation

##### B) Pure Pursuit - Saf Takip
```
Hız Vektörü → Her zaman hedefe yönelir
```
- Basit implementasyon
- "Tail-chase" durumu oluşturabilir
- Sabit kanatlı için minimum dönüş yarıçapı sorunu

##### C) Image-Based Visual Servoing (IBVS)
```
Görüntüdeki Hata → Doğrudan Kontrol Komutu
```
- 3B pozisyon hesabı gerektirmez
- Kamera görüntüsünden direkt komut üretir
- Sabit kanatlı için en uygun yöntemlerden biri

**Kaynak:** IEEE - Visual Servoing for Fixed-Wing UAVs

---

### 3. AIAA SUAS Competition Yaklaşımı

AUVSI SUAS yarışmasında kullanılan teknikler:

1. **Otonom Tespit + Sınıflandırma**: YOLO modeli ile hedef tespiti
2. **GPS Lokalizasyonu**: Tespit edilen hedefin GPS koordinatı hesabı
3. **Moving Target Tracking**: Lemniscate (∞ şekli) yolunda hareket eden hedef
4. **Pursuit + Avoidance**: Takip ederken çarpışmadan kaçınma

**Önemli Kaynaklar:**
- suas-competition.org - Resmi yarışma sayfası
- robonation.org - Teknik dokümanlar

---

### 4. Teknofest Savaşan İHA Yarışması

Türkiye'deki Teknofest yarışmasında:

- **Kamikaze İHA Görevi**: Hedefi tespit → Kilitleme → Dalış
- **Görüntü İşleme ile Tespit**: Rakip İHA'yı kamera ile algılama
- **Sanal Ortamda Kilitleme**: Gerçek çarpışma yok, simülasyon bazlı
- **Agresif Manevralar**: Kaçınma stratejileri

**Kaynak:** teknofest.org - Savaşan İHA

---

### 5. 4 Saniyelik Kesintisiz Takip için Önerilen Mimari

```python
# Önerilen Sistem Mimarisi

class AirToAirTracker:
    def __init__(self):
        self.detector = YOLO('yolov8n.pt')  # Nesne tespiti
        self.tracker = KalmanFilter()        # Pozisyon tahmini
        self.gimbal = GimbalController()     # Kamera kontrolü
        self.guidance = ProportionalNav(N=4) # Güdüm algoritması
        
    def tracking_loop(self):
        while True:
            # 1. Görüntü al
            frame = self.camera.get_frame()
            
            # 2. Hedef tespit et
            detection = self.detector.detect(frame)
            
            # 3. Tracker güncelle
            if detection:
                self.tracker.update(detection.center)
            predicted_pos = self.tracker.predict()
            
            # 4. Gimbal'ı hedefe yönelt
            gimbal_cmd = self.gimbal.point_to(predicted_pos)
            
            # 5. Uçak güdüm komutu hesapla (PN veya IBVS)
            guidance_cmd = self.guidance.compute(
                target_pos=predicted_pos,
                aircraft_heading=self.imu.heading
            )
            
            # 6. Uçağı yönlendir
            self.send_attitude_command(guidance_cmd)
```

---

### 6. Kritik Parametreler

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| **Kamera FPS** | ≥30 Hz | Kesintisiz takip için |
| **Detection Rate** | ≥20 Hz | YOLO inference süresi |
| **Gimbal Response** | <50ms | Hızlı stabilizasyon |
| **Guidance Loop** | ≥20 Hz | Kontrol frekansı |
| **Min Turn Radius** | Uçağa bağlı | Sabit kanat sınırlaması |

---

### 7. Kaynaklar ve Okuma Listesi

#### Akademik Makaleler:
1. **"Visual Servoing for Fixed-Wing UAV Target Tracking"** - IEEE Transactions on Aerospace
2. **"Proportional Navigation Guidance for UAV Trajectory Tracking"** - ResearchGate
3. **"YOLO-based Object Detection for UAV Air-to-Air Tracking"** - MDPI Drones
4. **"Image-Based Visual Servoing with Pan-Tilt Camera"** - ProQuest

#### Yarışma Kaynakları:
- AUVSI SUAS Competition Rules: https://suas-competition.org
- Teknofest Savaşan İHA Şartnamesi: https://teknofest.org
- AIAA Student Competition Guidelines: https://aiaa.org

#### GitHub Projeleri:
- YOLOv8 Object Tracking: `ultralytics/ultralytics`
- DeepSORT Tracker: `nwojke/deep_sort`
- ROS2 Visual Servoing: çeşitli projeler

---

---

## BÖLÜM 2: SABİT KAMERA İLE PID-TABANLI TAKİP SİSTEMİ

### Temel Mantık

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     SABİT KAMERA TAKİP SİSTEMİ                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   Kamera Görüntüsü                Hedef Pozisyon                       │
│   ┌─────────────────┐             (Kamerada)                           │
│   │    ┌───┐        │                                                   │
│   │    │ H │        │   ◄── Hedef burada görünüyor                     │
│   │    └───┘        │                                                   │
│   │                 │        error_x = hedef_x - merkez_x              │
│   │        ╋        │   ◄── Kamera merkezi (hedef burası olmalı)       │
│   │                 │        error_y = hedef_y - merkez_y              │
│   └─────────────────┘                                                   │
│                                                                         │
│   error_x > 0  →  Sağda  →  Sağa dön (Roll/Yaw)                        │
│   error_x < 0  →  Solda  →  Sola dön (Roll/Yaw)                        │
│   error_y > 0  →  Altta  →  Burnunu kaldır (Pitch)                     │
│   error_y < 0  →  Üstte  →  Burnunu indir (Pitch)                      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### PID Kontrol Döngüsü

```python
# Görüntü boyutları (piksel)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CENTER_X = IMAGE_WIDTH / 2    # 320
CENTER_Y = IMAGE_HEIGHT / 2   # 240

# PID Kazançları (tune edilmeli!)
Kp_yaw = 0.05      # Yaw için Proportional
Ki_yaw = 0.001     # Yaw için Integral
Kd_yaw = 0.02      # Yaw için Derivative

Kp_pitch = 0.03    # Pitch için Proportional
Ki_pitch = 0.001   # Pitch için Integral
Kd_pitch = 0.01    # Pitch için Derivative

class VisualTrackingPID:
    def __init__(self):
        # Yaw PID değişkenleri
        self.error_x_prev = 0
        self.error_x_integral = 0
        
        # Pitch PID değişkenleri
        self.error_y_prev = 0
        self.error_y_integral = 0
        
    def compute_control(self, target_x, target_y):
        """
        Kameradaki hedef pozisyonuna göre kontrol komutu hesapla.
        
        Args:
            target_x: Hedefin kameradaki X pozisyonu (piksel)
            target_y: Hedefin kameradaki Y pozisyonu (piksel)
            
        Returns:
            yaw_cmd: Yaw açısı değişimi (derece)
            pitch_cmd: Pitch açısı değişimi (derece)
        """
        # ============ YAW KONTROLÜ (Sağ/Sol) ============
        error_x = target_x - CENTER_X   # Pozitif = hedef sağda
        
        # PID hesabı
        self.error_x_integral += error_x
        error_x_derivative = error_x - self.error_x_prev
        
        yaw_cmd = (Kp_yaw * error_x + 
                   Ki_yaw * self.error_x_integral + 
                   Kd_yaw * error_x_derivative)
        
        self.error_x_prev = error_x
        
        # ============ PITCH KONTROLÜ (Yukarı/Aşağı) ============
        error_y = target_y - CENTER_Y   # Pozitif = hedef altta
        
        # PID hesabı
        self.error_y_integral += error_y
        error_y_derivative = error_y - self.error_y_prev
        
        # NOT: Y ekseni ters - hedef alttaysa burnunu kaldır (negatif pitch)
        pitch_cmd = -(Kp_pitch * error_y + 
                      Ki_pitch * self.error_y_integral + 
                      Kd_pitch * error_y_derivative)
        
        self.error_y_prev = error_y
        
        return yaw_cmd, pitch_cmd
```

---

### Tam Sistem Entegrasyonu (ROS2/MAVROS)

```python
#!/usr/bin/env python3
"""
Sabit Kamera ile Visual Servoing Takip Sistemi
Hedefin kameradaki pozisyonuna göre uçağı yönlendirir.
"""

class VisualTracker(Node):
    def __init__(self):
        super().__init__('visual_tracker')
        
        # ========== PARAMETRELER ==========
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CENTER_X = self.IMAGE_WIDTH / 2
        self.CENTER_Y = self.IMAGE_HEIGHT / 2
        
        # Dead zone - çok küçük hatalarda tepki verme
        self.DEADZONE_X = 30   # piksel
        self.DEADZONE_Y = 20   # piksel
        
        # PID Kazançları
        self.Kp_yaw = 0.05
        self.Ki_yaw = 0.001
        self.Kd_yaw = 0.02
        
        self.Kp_pitch = 0.03
        self.Ki_pitch = 0.001
        self.Kd_pitch = 0.01
        
        # Maksimum komut limitleri (güvenlik)
        self.MAX_YAW_RATE = 30.0     # derece/saniye
        self.MAX_PITCH_CMD = 20.0   # derece
        
        # PID state
        self.error_x_prev = 0
        self.error_x_integral = 0
        self.error_y_prev = 0
        self.error_y_integral = 0
        
        # Mevcut uçak durumu
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.base_thrust = 0.7  # Takip sırasındaki temel thrust
        
        # Hedef tespit edildi mi?
        self.target_detected = False
        self.target_x = 0
        self.target_y = 0
        self.tracking_time = 0.0  # 4 saniye sayacı
        
    def detection_callback(self, target_x, target_y, detected):
        """
        Görüntü işleme sisteminden hedef pozisyonu geldiğinde çağrılır.
        Bu callback YOLO veya başka bir detector'dan gelir.
        """
        self.target_detected = detected
        self.target_x = target_x
        self.target_y = target_y
        
    def compute_tracking_command(self):
        """
        PID kontrol komutu hesapla ve uçağa gönder.
        """
        if not self.target_detected:
            # Hedef kayboldu - arama moduna geç
            self.reset_pid()
            return None, None
        
        # ========== YAW (Sağ/Sol) ==========
        error_x = self.target_x - self.CENTER_X
        
        # Dead zone kontrolü
        if abs(error_x) < self.DEADZONE_X:
            error_x = 0
            
        self.error_x_integral += error_x
        # Anti-windup
        self.error_x_integral = max(-1000, min(1000, self.error_x_integral))
        
        error_x_derivative = error_x - self.error_x_prev
        
        yaw_cmd = (self.Kp_yaw * error_x + 
                   self.Ki_yaw * self.error_x_integral + 
                   self.Kd_yaw * error_x_derivative)
        
        # Limit
        yaw_cmd = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, yaw_cmd))
        self.error_x_prev = error_x
        
        # ========== PITCH (Yukarı/Aşağı) ==========
        error_y = self.target_y - self.CENTER_Y
        
        if abs(error_y) < self.DEADZONE_Y:
            error_y = 0
            
        self.error_y_integral += error_y
        self.error_y_integral = max(-1000, min(1000, self.error_y_integral))
        
        error_y_derivative = error_y - self.error_y_prev
        
        # Y ters - alttaysa kaldır
        pitch_cmd = -(self.Kp_pitch * error_y + 
                      self.Ki_pitch * self.error_y_integral + 
                      self.Kd_pitch * error_y_derivative)
        
        pitch_cmd = max(-self.MAX_PITCH_CMD, min(self.MAX_PITCH_CMD, pitch_cmd))
        self.error_y_prev = error_y
        
        return yaw_cmd, pitch_cmd
    
    def tracking_loop(self):
        """
        Ana takip döngüsü (50Hz).
        """
        yaw_cmd, pitch_cmd = self.compute_tracking_command()
        
        if yaw_cmd is not None:
            # Hedef merkeze yakınsa
            if abs(self.target_x - self.CENTER_X) < 50 and \
               abs(self.target_y - self.CENTER_Y) < 50:
                self.tracking_time += 0.02  # 50Hz = 20ms
                
                if self.tracking_time >= 4.0:
                    self.get_logger().info("✅ 4 SANİYE TAKİP TAMAMLANDI!")
            else:
                # Hedef merkezde değil, sayaç sıfırla
                self.tracking_time = 0.0
            
            # Attitude komutu gönder
            new_yaw = self.current_yaw + yaw_cmd * 0.02  # Δt = 20ms
            new_pitch = self.current_pitch + pitch_cmd
            
            self.publish_attitude(
                roll=0.0, 
                pitch=new_pitch, 
                yaw=new_yaw, 
                thrust=self.base_thrust
            )
            
            self.get_logger().info(
                f"Tracking | Error: ({self.target_x - self.CENTER_X:.0f}, "
                f"{self.target_y - self.CENTER_Y:.0f}) | "
                f"Cmd: yaw={yaw_cmd:.1f}°, pitch={pitch_cmd:.1f}° | "
                f"Time: {self.tracking_time:.1f}s/4.0s"
            )
    
    def reset_pid(self):
        """PID state'i sıfırla."""
        self.error_x_prev = 0
        self.error_x_integral = 0
        self.error_y_prev = 0
        self.error_y_integral = 0
```

---

### Sistem Diyagramı

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│   ┌─────────┐     ┌──────────┐     ┌─────────┐     ┌─────────┐  │
│   │ KAMERA  │ ──► │  YOLO    │ ──► │   PID   │ ──► │ MAVROS  │  │
│   │ (Sabit) │     │ Detector │     │ Control │     │ Attitude│  │
│   └─────────┘     └──────────┘     └─────────┘     └─────────┘  │
│        │                │                │               │       │
│        ▼                ▼                ▼               ▼       │
│   640x480          target_x,        yaw_cmd,        roll,pitch  │
│   frame            target_y         pitch_cmd       yaw,thrust  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

### Önemli Noktalar

1. **Dead Zone**: Hedef tam merkezde olmasa bile küçük hatalar için tepki verme - titreşimi önler

2. **Anti-Windup**: Integral terimini sınırla - aşırı birikim engelle

3. **Rate Limiting**: Ani komut değişikliklerini sınırla - uçak kararlılığı

4. **Thrust Kontrolü**: Takip sırasında sabit thrust kullanabilirsin, veya mesafeye göre ayarlayabilirsin

5. **Sabit Kanat Sınırlaması**: Minimum dönüş yarıçapı var - çok keskin dönüşler yapamaz

---

### Thrust ile Mesafe Kontrolü (Opsiyonel)

```python
def compute_thrust(self, target_area):
    """
    Hedefin kameradaki boyutuna göre thrust ayarla.
    Büyük görünüyorsa yaklaşmış demek - yavaşla.
    Küçük görünüyorsa uzaklaşmış demek - hızlan.
    """
    TARGET_AREA_IDEAL = 5000  # piksel kare (tune edilmeli)
    
    error_area = target_area - TARGET_AREA_IDEAL
    
    # Basit P kontrolü
    thrust_adjustment = -0.0001 * error_area
    
    thrust = self.base_thrust + thrust_adjustment
    thrust = max(0.4, min(1.0, thrust))  # Limit
    
    return thrust
```

---

### Sonraki Adımlar

1. **Görüntü işleme hangi frekansta çalışacak?** (en az 20Hz önerilir)
2. **PX4/MAVROS ile attitude kontrolü yapabiliyor musun şu an?** (qr2.py'de yapıyorsun)
3. **Hedef tespit algoritman hazır mı?** (YOLO, renk filtresi, vb.)

---

*Bu doküman 2026-02-06 tarihinde oluşturulmuştur.*
