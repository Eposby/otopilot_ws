# QR2.py Kodu - Kapsamlı Walkthrough

Bu doküman, `qr2.py` dosyasını en temelden anlamak için hazırlanmıştır. Her kavram adım adım açıklanmıştır.

---

## 📋 İÇİNDEKİLER

1. [Büyük Resim - Kod Ne Yapıyor?](#1-büyük-resim---kod-ne-yapıyor)
2. [ROS2 ve MAVROS Temelleri](#2-ros2-ve-mavros-temelleri)
3. [Koordinat Sistemleri](#3-koordinat-sistemleri)
4. [Konfigürasyon Parametreleri](#4-konfigürasyon-parametreleri)
5. [Sınıf Yapısı ve Değişkenler](#5-sınıf-yapısı-ve-değişkenler)
6. [State Machine (Durum Makinesi) Mantığı](#6-state-machine-durum-makinesi-mantığı)
7. [Matematiksel Fonksiyonlar](#7-matematiksel-fonksiyonlar)
8. [Görev Fonksiyonları](#8-görev-fonksiyonları)
9. [PX4 İletişim Fonksiyonları](#9-px4-iletişim-fonksiyonları)
10. [Hız Kontrolü Nasıl Eklenir?](#10-hız-kontrolü-nasıl-eklenir)

---

## 1. Büyük Resim - Kod Ne Yapıyor?

Bu kod bir **kamikaze saldırı missiyonu** gerçekleştiren otonom uçuş yazılımıdır.

### Görev Sırası (Mission Flow)

```
┌─────────────┐
│   TAKEOFF   │ ──► Yerden 100m'ye kalkış
└─────┬───────┘
      ▼
┌─────────────┐
│  GOTO_WP1   │ ──► QR hedefinin 320m gerisine git
└─────┬───────┘
      ▼
┌─────────────┐
│     WP2     │ ──► QR'a hızla yaklaş (200m kala dur)
└─────┬───────┘
      ▼
┌─────────────┐
│  KAMIKAZE   │ ──► Dalış yap + Pull-up ile kurtar
└─────┬───────┘
      ▼
┌─────────────┐
│  STABILIZE  │ ──► Attitude modundan çık, stabilize ol
└─────┬───────┘
      ▼
┌─────────────┐
│     RTL     │ ──► Eve dön (Return To Launch)
└─────┬───────┘
      ▼
┌─────────────┐
│    LAND     │ ──► İniş yap
└─────────────┘
```

### Görsel: Misyon Planı

```
                    HOME (Başlangıç)
                         │
                         │ 100m yükseklik
                         ▼
            WP1 ◄────── 320m ──────► QR Hedefi
             │                          │
             │         150m alt.        │
             │                          │
             └─────────────────────────►│
                   TURBO APPROACH       │
                   (200m'de dur)        ▼
                                    KAMIKAZE
                                    (Dalış)
                                        │
                                        ▼
                                    PULL-UP
                                    (60m'de)
```

---

## 2. ROS2 ve MAVROS Temelleri

### ROS2 Nedir?

ROS2 (Robot Operating System 2), robotlar arasında iletişim sağlayan bir middleware'dir. Ana konseptler:

| Kavram | Açıklama | Örnek |
|--------|----------|-------|
| **Node** | Bağımsız çalışan bir program | `QR2` sınıfı bir node |
| **Topic** | Mesaj yayınlama/dinleme kanalı | `/mavros/local_position/pose` |
| **Publisher** | Topic'e mesaj gönderen | `self.local_pos_pub` |
| **Subscriber** | Topic'i dinleyen | `self.local_pos_sub` |
| **Service** | İstek-Cevap iletişimi | `/mavros/cmd/arming` |
| **Timer** | Periyodik fonksiyon çağrısı | `self.timer` (50ms) |

### MAVROS Nedir?

MAVROS, ROS2 ile PX4/ArduPilot arasında köprü görevi görür:

```
┌─────────────────┐     ROS2 Topics     ┌─────────────┐     MAVLink     ┌────────────┐
│   Python Kodu   │ ◄─────────────────► │   MAVROS    │ ◄────────────► │    PX4     │
│     (qr2.py)    │                     │             │                │ (Autopilot)│
└─────────────────┘                     └─────────────┘                └────────────┘
```

### QoS Profile

```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Hız için güvenilirliği azalt
    history=HistoryPolicy.KEEP_LAST,            # Sadece son mesajları tut
    depth=10                                     # Son 10 mesajı hafızada tut
)
```

> [!NOTE]
> `BEST_EFFORT` kullanılmasının sebebi: Sensör verileri çok hızlı gelir, kayıp birkaç mesaj önemli değildir. `RELIABLE` kullanılsaydı, gecikme olurdu.

---

## 3. Koordinat Sistemleri

Bu kod **iki farklı koordinat sistemi** kullanır:

### 3.1 Global Koordinatlar (GPS)

- **Latitude (Enlem)**: Kuzey-Güney pozisyonu (derece)
- **Longitude (Boylam)**: Doğu-Batı pozisyonu (derece)
- **Altitude**: Deniz seviyesinden yükseklik (metre)

```python
QR_LAT = 40.230712658763466  # Hedefin enlem'i
QR_LON = 29.006760026391138  # Hedefin boylam'ı
```

### 3.2 Local ENU Koordinatları

PX4 **ENU (East-North-Up)** koordinat sistemi kullanır:

```
        Y (Kuzey/North)
        ▲
        │
        │
        │
        └──────────► X (Doğu/East)
       /
      /
     ▼
    Z (Yukarı/Up)
```

| Eksen | Yön | Birim |
|-------|-----|-------|
| X | Doğu (+) / Batı (-) | Metre |
| Y | Kuzey (+) / Güney (-) | Metre |
| Z | Yukarı (+) / Aşağı (-) | Metre |

### 3.3 GPS → Local Dönüşüm

```python
def global_to_local(self, lat, lon):
    R = 6371000.0  # Dünya yarıçapı (metre)
    
    # Latitude farkı → Y (Kuzey) mesafesi
    d_lat = math.radians(lat - self.home_pos.latitude)
    y = d_lat * R  # metre
    
    # Longitude farkı → X (Doğu) mesafesi
    d_lon = math.radians(lon - self.home_pos.longitude)
    lat_mean = math.radians(self.home_pos.latitude)
    x = d_lon * R * math.cos(lat_mean)  # metre
    
    return x, y
```

**Matematiksel Açıklama:**

1. **Neden `R * d_lat`?** Çünkü yay uzunluğu = yarıçap × açı (radyan)
2. **Neden `cos(lat_mean)`?** Longitude çemberi ekvatorda en geniş, kutuplarda sıfırdır. Enlem arttıkça küçülür.

```
Ekvatorda (lat=0°):   cos(0°) = 1.0    → Tam mesafe
Türkiye'de (lat=40°): cos(40°) = 0.766 → %76.6 mesafe
Kutupta (lat=90°):    cos(90°) = 0.0   → Sıfır mesafe
```

---

## 4. Konfigürasyon Parametreleri

### 4.1 Konum Parametreleri

```python
QR_LAT = 40.230712658763466  # Hedef enlem (derece)
QR_LON = 29.006760026391138  # Hedef boylam (derece)
TAKEOFF_ALT = 100.0          # Kalkış irtifası (metre)
WP1_ALT = 150.0              # Waypoint 1 irtifası (metre)
WP1_DISTANCE = 320.0         # QR'ın gerisinde WP1 mesafesi (metre)
WP2_DISTANCE = 200.0         # Kamikaze başlangıç mesafesi (metre)
PULL_UP_ALT = 60.0           # Pull-up başlangıç irtifası (metre)
SAFE_ALT = 100.0             # Güvenli irtifa (metre)
```

### 4.2 Uçuş Profili Parametreleri

```python
# Dalış (DIVE) Parametreleri
DIVE_PITCH_1 = 40.0   # Phase 1: Yumuşak dalış (150m→130m)
DIVE_PITCH_2 = 40.0   # Phase 2: Orta dalış (130m→110m)
DIVE_PITCH_3 = 50.0   # Phase 3: Dik dalış (110m→65m)
DIVE_THRUST = 0.8     # Dalış sırasında motor gücü (%80)

# Kurtarma (PULL-UP) Parametreleri
PULLUP_PITCH_1 = -30.0  # 60-70m arası (yukarı pitch)
PULLUP_PITCH_2 = -20.0  # 70-80m arası
PULLUP_PITCH_3 = -10.0  # 80-90m arası
PULLUP_PITCH_4 = 0.0    # 90m+ düz uçuş
PULLUP_ROLL = 45.0      # Kaçış roll açısı (sağa/sola dön)
PULLUP_THRUST = 1.0     # Pull-up sırasında tam güç (%100)
```

### Pitch Açısı Nedir?

```
Pitch Pozitif (+40°) = Burun aşağı (dalış)
        ↘
         ╲
          ╲
           ╲▼
           
Pitch Negatif (-30°) = Burun yukarı (tırmanış)
           ▲╱
          ╱
         ╱
        ↗
        
Pitch Sıfır (0°) = Düz uçuş
        ────────────►
```

> [!IMPORTANT]
> PX4'te sabit kanatlı uçaklar için pozitif pitch = burun aşağı. Bu multirotor'un tersidir!

---

## 5. Sınıf Yapısı ve Değişkenler

### 5.1 Ana Sınıf

```python
class QR2(Node):
    def __init__(self):
        super().__init__('qr2_node')  # ROS2 node adı
```

### 5.2 Subscriber'lar (Veri Okuma)

```python
# 1. PX4 durumu (armed, mode, connected)
self.state_sub = self.create_subscription(
    State, '/mavros/state', self.state_callback, qos_profile)

# 2. GPS konumu (latitude, longitude, altitude)
self.global_pos_sub = self.create_subscription(
    NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos_profile)

# 3. Local konum (x, y, z metre cinsinden)
self.local_pos_sub = self.create_subscription(
    PoseStamped, '/mavros/local_position/pose', self.local_position_callback, qos_profile)

# 4. Home pozisyonu (origin)
self.home_sub = self.create_subscription(
    HomePosition, '/mavros/home_position/home', self.home_position_callback, qos_profile)
```

### 5.3 Publisher'lar (Komut Gönderme)

```python
# 1. Konum komutu (git şu noktaya)
self.local_pos_pub = self.create_publisher(
    PoseStamped, '/mavros/setpoint_position/local', 10)

# 2. Attitude komutu (şu açıda uç)
self.attitude_pub = self.create_publisher(
    AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
```

### 5.4 Service Client'lar (Özel Komutlar)

```python
# 1. Arm/Disarm (motorları aç/kapat)
self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

# 2. Takeoff (kalkış)
self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

# 3. Mod değiştir (OFFBOARD, AUTO.RTL, AUTO.LAND)
self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
```

### 5.5 Durum Değişkenleri

```python
# PX4'ten gelen anlık durum
self.current_state = State()           # armed, mode, connected
self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Anlık konum
self.home_pos = None                   # GPS home konumu
self.mavros_origin_set = False         # Origin ayarlandı mı?
self.home_alt_amsl = 0.0               # Home deniz seviyesi irtifası

# Görev yönetimi
self.mission_queue = []                # Görev listesi
self.current_task = None               # Şu anki görev
self.task_state = 0                    # Görev içi alt-durum

# Hesaplanan değerler
self.qr_x = 0.0                        # Hedef X (local)
self.qr_y = 0.0                        # Hedef Y (local)
self.wp1_x = 0.0                       # WP1 X (local)
self.wp1_y = 0.0                       # WP1 Y (local)

# Yardımcı sayaçlar
self.log_counter = 0                   # Log spam önleme
self.timeout_counter = 0               # Zamanlayıcı
```

---

## 6. State Machine (Durum Makinesi) Mantığı

### 6.1 İki Seviyeli State Machine

Bu kod **iç içe iki state machine** kullanır:

```
SEVIYE 1: Görevler (mission_queue)
┌─────────────────────────────────────────────────┐
│ TAKEOFF → GOTO_WP1 → WP2 → KAMIKAZE → RTL → LAND │
└─────────────────────────────────────────────────┘
                        │
                        ▼
SEVIYE 2: Alt-durumlar (task_state)
┌───────────────────────────────────────────────────┐
│ Her görevin kendi iç fazları var                  │
│ Örnek KAMIKAZE: 0→1→2→3→4→5→6 (7 faz)             │
└───────────────────────────────────────────────────┘
```

### 6.2 Control Loop (Ana Döngü)

```python
def control_loop(self):
    # Her 50ms'de bir çalışır (20 Hz)
    
    # 1. MAVROS hazır mı kontrol et
    if not self.home_pos or not self.mavros_origin_set:
        return  # Bekle
    
    # 2. Görev kuyruğu boşsa, görevleri oluştur
    if len(self.mission_queue) == 0 and self.current_task is None:
        self.setup_mission()
    
    # 3. Aktif görev yoksa, kuyruktan al
    if self.current_task is None:
        if len(self.mission_queue) > 0:
            self.current_task = self.mission_queue.pop(0)
            self.task_state = 0  # Sıfırdan başla
    
    # 4. Görevi çalıştır
    task_type = self.current_task['type']
    if task_type == 'TAKEOFF':
        self.execute_takeoff()
    elif task_type == 'KAMIKAZE':
        self.execute_kamikaze()
    # ... diğer görevler
```

### 6.3 Task State Örneği: TAKEOFF

```python
def execute_takeoff(self):
    target_alt = self.current_task['alt']
    
    if self.task_state == 0:
        # İLK ÇAĞRI: Takeoff komutu gönder
        self.send_takeoff_command(target_alt)
        self.task_state = 1  # Sonraki duruma geç
    
    elif self.task_state == 1:
        # BEKLE: AUTO.TAKEOFF moduna geçmesini bekle
        if self.current_state.mode == "AUTO.TAKEOFF":
            self.send_arm_command(True)  # Arm et
            self.task_state = 2
    
    elif self.task_state == 2:
        # TIRMANIŞ: İrtifa hedefine ulaşmayı bekle
        if self.current_pos['z'] > (target_alt - 5.0):
            self.finish_task()  # Görevi tamamla
```

### State Geçiş Diyagramı: TAKEOFF

```
┌─────────────────┐
│  task_state = 0 │
│  Takeoff komutu │
└────────┬────────┘
         │ Komut gönderildi
         ▼
┌─────────────────┐
│  task_state = 1 │
│  Mode bekleniyor│
└────────┬────────┘
         │ mode == "AUTO.TAKEOFF"
         ▼
┌─────────────────┐
│  task_state = 2 │
│  İrtifa bekle   │
└────────┬────────┘
         │ z > (target_alt - 5)
         ▼
┌─────────────────┐
│  finish_task()  │
│  Sonraki göreve │
└─────────────────┘
```

---

## 7. Matematiksel Fonksiyonlar

### 7.1 Mesafe Hesabı (Pisagor)

```python
def distance_to(self, x, y):
    return math.sqrt((x - self.current_pos['x'])**2 + 
                     (y - self.current_pos['y'])**2)
```

**Görsel:**
```
       Hedef (x, y)
         ●
        /│
       / │
      /  │ dy = y - current_y
     /   │
    /    │
   ●─────┘
Mevcut   dx = x - current_x
(current_x, current_y)

mesafe = √(dx² + dy²)
```

### 7.2 Yaw Açısı Hesabı

```python
def yaw_to(self, x, y):
    return math.degrees(math.atan2(
        y - self.current_pos['y'],  # dy
        x - self.current_pos['x']   # dx
    ))
```

**atan2 Nedir?**

`atan2(y, x)` fonksiyonu, (0,0)'dan (x,y)'ye olan vektörün açısını döndürür.

```
        90°
         │
         │
180° ────┼──── 0°
         │
         │
       -90°
```

**Örnek:**
- Hedef doğuda → `atan2(0, 1)` = 0°
- Hedef kuzeyde → `atan2(1, 0)` = 90°
- Hedef batıda → `atan2(0, -1)` = 180°
- Hedef güneyde → `atan2(-1, 0)` = -90°

### 7.3 Birim Vektör ve WP1 Hesabı

```python
# QR'a olan mesafe vektörü
dx = self.qr_x - self.current_pos['x']
dy = self.qr_y - self.current_pos['y']
dist = math.sqrt(dx**2 + dy**2)

# Birim vektör (uzunluk = 1)
ux, uy = dx / dist, dy / dist

# WP1 = QR'ın 320m gerisinde
self.wp1_x = self.qr_x - ux * WP1_DISTANCE
self.wp1_y = self.qr_y - uy * WP1_DISTANCE
```

**Görsel:**
```
                          dx, dy
            ┌─────────────────────────────────►
            │                                  ● QR
            │                         ▲
            │                         │
            │                         │ 320m
            │                         │
            │                         ▼
            ● ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ● WP1
         Mevcut                    (QR - 320*unit_vector)
```

### 7.4 Euler → Quaternion Dönüşümü

3D rotasyonları temsil etmek için **quaternion** kullanılır.

**Euler Açıları:**
- **Roll**: X ekseni etrafında dönme (kanat sallama)
- **Pitch**: Y ekseni etrafında dönme (burun yukarı/aşağı)
- **Yaw**: Z ekseni etrafında dönme (pusula yönü)

```python
def euler_to_quaternion(self, roll, pitch, yaw):
    # Yarım açıların sinüs/kosinüsü
    cr = math.cos(roll / 2);  sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2);   sy = math.sin(yaw / 2)
    
    # Quaternion bileşenleri
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return x, y, z, w
```

> [!NOTE]
> Quaternion neden kullanılır? Euler açılarında **gimbal lock** problemi vardır. Quaternion bu sorunu çözer ve daha verimli interpolasyon sağlar.

---

## 8. Görev Fonksiyonları

### 8.1 execute_goto_wp1 - Waypoint'e Git

```python
def execute_goto_wp1(self):
    if self.task_state == 0:
        # 1. Önce mevcut konumu setpoint gönder (OFFBOARD için gerekli)
        self.publish_setpoint(self.current_pos['x'], self.current_pos['y'], self.current_pos['z'])
        
        # 2. OFFBOARD moduna geç
        self.set_mode_command("OFFBOARD")
        self.task_state = 1
    
    elif self.task_state == 1:
        # 3. Hedef konumu sürekli gönder (20 Hz)
        self.publish_setpoint(self.wp1_x, self.wp1_y, WP1_ALT)
        
        # 4. Mesafeyi kontrol et
        dist = self.distance_to(self.wp1_x, self.wp1_y)
        
        if dist < 50.0:
            self.finish_task()  # 50m'den yakınsa tamamla
```

> [!IMPORTANT]
> **OFFBOARD modu için önemli**: PX4, OFFBOARD'a girmeden ÖNCE en az bir setpoint almış olmalı. Yoksa güvenlik moduna geçer!

### 8.2 execute_turbo_approach - Hızlı Yaklaşma

```python
def execute_turbo_approach(self):
    # QR'ın ilerisine bir nokta hesapla (overshoot)
    overshoot_x = self.qr_x + (self.qr_x - self.wp1_x) * 1.0
    overshoot_y = self.qr_y + (self.qr_y - self.wp1_y) * 1.0
    
    # Bu noktaya git (QR'dan geçecek)
    self.publish_setpoint(overshoot_x, overshoot_y, WP1_ALT)
    
    # QR'a mesafeyi kontrol et
    dist = self.distance_to(self.qr_x, self.qr_y)
    
    if dist < WP2_DISTANCE:  # 200m
        self.finish_task()  # Kamikaze'ye geç
```

**Overshoot Nedir?**

```
     WP1 ────────────► QR ────────────► Overshoot
      ●                 ●                  ●
      │                 │                  │
      │                 │                  │
      └─────── 320m ────┴────── 320m ──────┘
      
Uçak, overshoot'a gitmek için QR'dan geçmek zorunda.
Bu sayede QR'a tam hız ile yaklaşır.
```

### 8.3 execute_kamikaze - Dalış ve Kurtarma

Bu en karmaşık görevdir. 7 ayrı fazı vardır:

```
DIVE (Dalış) Fazları:
├── Phase 0: Yumuşak dalış (150m → 130m) @ 40° pitch
├── Phase 1: Orta dalış (130m → 110m) @ 40° pitch
└── Phase 2: Dik dalış (110m → 65m) @ 50° pitch

PULL-UP (Kurtarma) Fazları:
├── Phase 3: Agresif pull-up (60m → 70m) @ -30° pitch, 45° roll
├── Phase 4: Orta pull-up (70m → 80m) @ -20° pitch, 45° roll
├── Phase 5: Yumuşak pull-up (80m → 90m) @ -10° pitch, 45° roll
└── Phase 6: Düz uçuş (90m → 100m) @ 0° pitch
```

```python
def execute_kamikaze(self):
    yaw = self.yaw_to(self.qr_x, self.qr_y)  # QR'a yönel
    
    if self.task_state == 0:  # DIVE Phase 1
        self.publish_attitude(roll=0.0, pitch=40.0, yaw=yaw, thrust=0.8)
        if self.current_pos['z'] < 130:
            self.task_state = 1
    
    elif self.task_state == 3:  # PULL-UP Phase 1
        self.publish_attitude(roll=45.0, pitch=-30.0, yaw=yaw, thrust=1.0)
        if self.current_pos['z'] > 70:
            self.task_state = 4
    
    # ... diğer fazlar
```

---

## 9. PX4 İletişim Fonksiyonları

### 9.1 publish_setpoint - Konum Komutu

```python
def publish_setpoint(self, x, y, z):
    msg = PoseStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "map"  # ENU koordinat sistemi
    
    msg.pose.position.x = float(x)  # Doğu (metre)
    msg.pose.position.y = float(y)  # Kuzey (metre)
    msg.pose.position.z = float(z)  # Yukarı (metre)
    msg.pose.orientation.w = 1.0    # Varsayılan yaw
    
    self.local_pos_pub.publish(msg)
```

> [!TIP]
> Bu fonksiyon sadece **konum** gönderir. PX4 hızı kendisi hesaplar. Eğer hızı kontrol etmek istersen, `PositionTarget` mesajı kullanmalısın.

### 9.2 publish_attitude - Uçuş Açısı Komutu

```python
def publish_attitude(self, roll, pitch, yaw, thrust):
    msg = AttitudeTarget()
    msg.header.stamp = self.get_clock().now().to_msg()
    
    # Angular rate'leri yoksay, sadece orientation kullan
    msg.type_mask = 7  # IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE
    
    # Euler → Quaternion dönüşümü
    x, y, z, w = self.euler_to_quaternion(
        math.radians(roll),
        math.radians(pitch),
        math.radians(yaw)
    )
    
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z
    msg.orientation.w = w
    
    msg.thrust = float(thrust)  # 0.0 - 1.0 arası
    
    self.attitude_pub.publish(msg)
```

### Type Mask Nedir?

`type_mask` hangi parametrelerin yoksayılacağını belirtir:

| Bit | Değer | Açıklama |
|-----|-------|----------|
| 0 | 1 | IGNORE_ROLL_RATE |
| 1 | 2 | IGNORE_PITCH_RATE |
| 2 | 4 | IGNORE_YAW_RATE |
| 6 | 64 | IGNORE_THRUST |
| 7 | 128 | IGNORE_ATTITUDE |

`type_mask = 7` → 1 + 2 + 4 = Angular rate'leri yoksay, orientation ve thrust kullan.

---

## 10. Hız Kontrolü Nasıl Eklenir?

Şu an kod **sadece konum** gönderiyor. Hız kontrolü için iki seçenek var:

### Seçenek 1: PositionTarget Mesajı (Önerilen)

```python
from mavros_msgs.msg import PositionTarget

# Publisher ekle
self.setpoint_raw_pub = self.create_publisher(
    PositionTarget, '/mavros/setpoint_raw/local', 10)

def publish_position_with_velocity(self, x, y, z, vx=None, vy=None, vz=None):
    msg = PositionTarget()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    
    # Konum
    msg.position.x = x
    msg.position.y = y
    msg.position.z = -z  # NED: Aşağı pozitif
    
    if vx is not None and vy is not None:
        # Hız limiti uygula
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz if vz else 0.0
        msg.type_mask = 0b0000_0000_0111  # Konum + Hız kullan
    else:
        msg.type_mask = 0b0000_1111_1000  # Sadece konum kullan
    
    self.setpoint_raw_pub.publish(msg)
```

### Seçenek 2: Mesafe Bazlı Dinamik Hız

```python
def execute_goto_wp1_with_speed_control(self):
    dist = self.distance_to(self.wp1_x, self.wp1_y)
    
    # Hız hesapla
    if dist > 100.0:  # 100m'den uzak
        target_speed = 25.0  # Tam hız
    else:
        target_speed = 15.0  # Yavaşla
    
    # Yön vektörü
    dx = self.wp1_x - self.current_pos['x']
    dy = self.wp1_y - self.current_pos['y']
    
    # Birim vektör × hız = hız vektörü
    if dist > 0:
        vx = (dx / dist) * target_speed
        vy = (dy / dist) * target_speed
    else:
        vx, vy = 0.0, 0.0
    
    self.publish_position_with_velocity(
        self.wp1_x, self.wp1_y, WP1_ALT, 
        vx, vy, 0.0
    )
```

---

## 📊 Özet Tablo

| Fonksiyon | Girdi | Çıktı | Açıklama |
|-----------|-------|-------|----------|
| `global_to_local(lat, lon)` | GPS (derece) | (x, y) metre | Koordinat dönüşümü |
| `distance_to(x, y)` | Hedef konum | Metres | 2D mesafe |
| `yaw_to(x, y)` | Hedef konum | Derece | Hedefe yön açısı |
| `euler_to_quaternion(r, p, y)` | Radyan açılar | (x, y, z, w) | Rotasyon dönüşümü |
| `publish_setpoint(x, y, z)` | Metre konum | - | PX4'e konum gönder |
| `publish_attitude(r, p, y, t)` | Açılar + thrust | - | PX4'e attitude gönder |

---

> [!TIP]
> Bu dokümanı referans olarak kullan. Kodu değiştirirken önce ilgili bölümü oku ve matematiksel mantığı anla.
