# Savaşan İHA Otonom Uçuş Sistemleri (otopilot_ws)

Bu repo, TEKNOFEST Savaşan İHA yarışması otonom görevler ve uçuş kontrolü için geliştirilmiş **kapsamlı, modüler ve otonom bir İHA (İnsansız Hava Aracı) yönetim sistemidir.** ROS 2 (Humble) ve MAVROS (PX4) tabanlı olarak çalışır. 

Sistem sadece basit bir otopilot değil; görüntü işleme, dinamik hedef takibi, görev yönetimi (state machine), telemetri izleme ve acil durum protokollerini içeren gelişmiş bir otonomi çatısıdır.

---

## Temel Özellikler ve Modüller

Proje, "Sense-Think-Act" (Algıla-Düşün-Uygula) mimarisine uygun olarak farklı paketlere/düğümlere (node) bölünmüştür. Her bir görev için özel yazılmış bağımsız algoritmalar bulunur.

### 1. Beyin ve Karar Mekanizmaları (`iha_görev`)
Yüksek seviyeli mantık ve yapay zeka operasyonlarının yürütüldüğü paket.
- **Mission Commander:** Alt görevler (Kalkış, Devriye, Hedef Takibi, İniş) arası geçişleri state machine (durum makinesi) ile yöneten sistemin ana yöneticisi.
- **Safety Watchdog:** Uçağın hız, irtifa, batarya ve MAVLink bağlantısını saniye saniye izleyip tehlike anında (Örn: İrtifa kaybı) acil durum (RTL - Return To Launch) başlatan güvenlik sistemi.
- **Telemetry Monitor:** GCS (Yer Kontrol İstasyonu) verilerini analiz edip durum raporlaması yapan sistem.
- **Trajectory Generator:** Dinamik hedefleri vurmak veya takip etmek için optimum uçuş rotalarının (eğrilerinin) matematiksel olarak anlık hesaplanması.

### 2. Gelişmiş Bilgisayarlı Görü (`iha_görev / vision`)
Kamera ve yapay zeka entegrasyonu.
- **Vision Processor / QR Vision Node:** Gazebo'dan veya gerçek kameradan gelen görüntüyü süzerek üzerine YOLOv8 modelini uygular. Hedefleri (Düşman İHA, Yer Hedefi, QR Panosu vb.) tespit edip "TargetInfo" mesajları ile sisteme yayar.
- **Kalman Tracker:** Hedef tespiti anlık kopsa bile, hedefin önceki hız/yön vektörlerini kullanarak nereye gidebileceğini tahmin eden ve uçağın hedefi kaybetmesini önleyen takip algoritması.
- **Async QR Reader:** Asenkron işlemci çekirdeklerinde (multiprocessing) çalışarak FPS kaybı yaşatmadan yüksek süratte hedeflerdeki şifreli QR kodları okuyan modül.
- **Camera Bridge:** Gazebo GUI kullanılmasa bile simülasyondaki kameranın raw görüntüsünü en düşük gecikmeyle (BEST_EFFORT QoS) ROS 2'ye taşıyan aktarıcı.

### 3. Uçuş ve Görev İcrası (`iha_otopilot_mavros`)
Beyinden gelen "Şunu yap" komutlarını MAVLink sinyallerine çevirip PX4 Otopilot'a ileten "Sinir Sistemi".
- **QR Kamikaze (qr.py):** Hedef tespit edildiğinde uçağın hızı ve otopilot PID katsayılarını dinamik olarak güncelleyerek, görsel geri bildirimle (Visual Servoing) hedefi kameranın tam merkezinde tutarak dalış yapan yazılım.
- **HSS Waypoint Mission:** Hava Savunma Sistemi'nden/Rakiplerden kaçınarak belirli GPS koordinatlarında 3 boyutlu manevralı uçuşlar yapmayı sağlayan görev nodu.
- **Enemy Plane Track:** İt dalaşı görevleri için rakipleri arkadan veya üstten takip pozisyonuna girmek amacıyla yazılmış takip (dogfight) algoritması.
- **Dynamic Mission Manager:** Uçuş sırasında sahadan gelen anlık bildirimlere göre İHA'nın görev planını (Örn: Devriyeyi yarıda kes ve düşmana kilitlen) havada değiştiren modül.

### 4. İstemci - Sunucu Haberleşmesi
Sürü İHA veya Yer İstasyonu haberleşmeleri için.
- **Emir Client / Server (`komut_client`, `komut_server`):** Gelişmiş ROS 2 Service / Action yapısıyla uçak dışından (Yapay Zeka veya Yer İstasyonu tabanlı) komut alıp - sonuç döndüren yapı.

---

## Dosya ve Paket Yapısı

```
otopilot_ws/
├── src/
│   ├── iha_otopilot/              (Python kodlarının bulunduğu ana paket)
│   │   ├── iha_görev/             (Yapay zeka, takip, güvenlik sistemleri)
│   │   ├── iha_otopilot_mavros/   (Uçuş komutları, MAVROS iletişimi)
│   │   ├── models/                (YOLO Ağırlıkları .pt, OpenCV QR Modelleri .caffemodel)
│   │   └── setup.py               (Node ve Entry-Point kayıtları)
│   │
│   └── iha_messages/              (Haberleşme protokolleri)
│       └── msg/
│           └── TargetInfo.msg     (Hedef pikselleri, hata payları, QR sonuçları)
```

---

## Kurulum ve Gereksinimler

Projenin derlenmesi ve çalışması için aşağıdaki kütüphanelerin yüklü olması gerekir.

```bash
# ROS 2 Humble ve MAVROS sisteminizde kurulu olmalıdır.

# Python Derin Öğrenme ve Görüntü İşleme Bağımlılıkları
pip install ultralytics opencv-contrib-python numpy pyzbar requests

# C++ QR Engine sistem kütüphanesi
sudo apt-get install -y libzbar0
```

---

## Nasıl Çalıştırılır?

Bu çalışma alanı doğrudan bir simülasyon ortamıyla (Gazebo - `sartek_ws`) veya gerçek PX4 otopilotu bağlı donanımla çalışmaya hazırdır.

### 1. Çalışma Alanını Derleyin
Herhangi bir Python dosyasına veya mesaja ekleme yaptıktan sonra:
```bash
cd ~/otopilot_ws
colcon build --packages-select iha_messages
colcon build --packages-select iha_otopilot
source install/setup.bash
```

### 2. Node Örnekleri (Kullanım Senaryoları)

Önce terminalden `source install/setup.bash` yapılmalıdır.

**A) Savaşan İHA - İt Dalaşı (Dogfight) Görevi Başlatmak:**
```bash
ros2 run iha_otopilot enemy_plane
ros2 run iha_otopilot vision_processor
```

**B) Kamikaze ve QR Hedef Okuma Başlatmak:**
Kamerayı yayına alıp (siyah ekran veya Gazebo), ardından YOLO beyin node'unu başlatıp, son olarak uçuşu tetikler.
```bash
ros2 run iha_otopilot camera_bridge
ros2 run iha_otopilot qr_vision --ros-args -p camera_topic:=/camera/image_raw
ros2 run iha_otopilot qr
```

**C) Telemetri ve Otonom Sağlık Durumu Kontrolü:**
```bash
ros2 run iha_otopilot safety_watchdog
ros2 run iha_otopilot telemetry_monitor
```

**(Not: Tüm düğümler listesini ve kısa komutlarını `setup.py` dosyasındaki `entry_points` kısmında bulabilirsiniz.)**
