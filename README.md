# Otopilot Çalışma Alanı (otopilot_ws)

Bu repo, TEKNOFEST Savaşan İHA yarışması için geliştirilmiş otonom uçuş, hedef tespiti, QR okuma ve görev yönetim sistemini içerir. ROS 2 (Humble) ve MAVROS tabanlı olarak çalışır.

Kod yapısı ve hiyerarşisi hala geliştirme aşamasındadır !!!
## Özellikler

- **Kamikaze Görev Yöneticisi:** `qr2.py` üzerinden state machine ile otonom kalkış, hedef arama, dalış, pull-up ve iniş.
- **Hedef Tespiti (YOLOv8):** Özel eğitilmiş YOLOv8 modeli ile uçak/hedef tespiti.
- **Dinamik Hedef Takibi:** Kalman Filtresi ile kamera merkezli PID uçuş kontrolü (hedef vizyondan çıksa bile hıza dayalı tahminsel takip).
- **Asenkron QR Okuyucu:** Multiprocessing ve WeChat QR kütüphanesi kullanılarak hedefe kilitlendiğinde QR kodun okunması.
- **Otomatik Hız Kontrolü:** Uçağın hedefe yaklaştıkça `FW_AIRSPD_MAX` parametresi üzerinden dinamik olarak yavaşlatılması.

##  Paket Yapısı (iha_otopilot)

Ana kodlar `src/iha_otopilot/` dizini altındadır:

*   **`iha_görev/` (Gözler ve Beyin)**
    *   `qr_vision_node.py`: ROS 2 kamera topiğini dinler, YOLO ve QR algoritmalarını çalıştırıp `TargetInfo` mesajı yayınlar.
    *   `qr_tracker.py`: Kalman filtresi sınıfı.
    *   `qr_reader.py`: Asenkron çalışan yüksek performanslı QR kodu çözücü.
    *   `mission_commander.py`: Görevler arası geçişi yöneten üst sistem.
    *   `models/`: `best.pt` (YOLO) ve `.caffemodel` (QR) yapay zeka ajanları.
*   **`iha_otopilot_mavros/` (Sinir Sistemi)**
    *   `qr2.py`: İHA'ya MAVLink komutları gönderen asıl kontrolcü.
    *   `gz_bridge_camera.py`: Gazebo'dan yüksek performanslı RAW görüntü aktarıcı.
*   **`iha_messages/`**
    *   `TargetInfo.msg`: Kamera node'undan uçuş node'una gönderilen standart veri tipi.

##  Kurulum (Kurulum Öncesi Gereksinimler)

Sistemin çalışması için ROS 2 Humble ve MAVROS'un sisteminizde yüklü olması gerekir.

```bash
# Python gereksinimleri
pip install ultralytics opencv-contrib-python numpy pyzbar requests

# Sistem gereksinimleri (QR kütüphanesi için)
sudo apt-get install -y libzbar0
```

##  Nasıl Çalıştırılır?

Bu çalışma alanı doğrudan `sartek_ws` (simülasyon) ile paralel çalışacak şekilde tasarlanmıştır.

### 1. Çalışma Alanını Derleyin
```bash
cd ~/otopilot_ws
colcon build --packages-select iha_messages
colcon build --packages-select iha_otopilot
source install/setup.bash
```

### 2. Düğümleri (Nodes) Başlatın
**Gazebo Kamera Köprüsü:**
```bash
ros2 run iha_otopilot camera_bridge
```

**Bilgisayarlı Görü (QR Vision):**
Gazebo kamerasına bağlanıp ekranda FPS ve hedef bilgilerini gösterecek node:
```bash
ros2 run iha_otopilot qr_vision
```
*(Eğer kamera topiğiniz farklıysa şöyle değiştirebilirsiniz: `ros2 run iha_otopilot qr_vision --ros-args -p camera_topic:=/sizin_topic`)*

**Kamikaze Uçuş Kontrolcüsü:**
Uçağı otonom olarak uçurup dalış manevralarını yapacak node:
```bash
ros2 run iha_otopilot qr
```

## Veri Akışı (Topic'ler)
*   **Dinlenen Topic:** `/camera/image_raw` (sensor_msgs/Image)
*   **Yayınlanan TargetInfo:** `/gorev/qr_target_info`
*   **Okunan QR Metni:** `/gorev/qr_result` (std_msgs/String)

---
*Geliştirici Notu: Bu kodları doğrudan ana simülasyon sisteminize kopyalamak yerine, ROS 2'nin doğasına uygun olarak bağımsız bir workspace olarak çalıştırın!*
