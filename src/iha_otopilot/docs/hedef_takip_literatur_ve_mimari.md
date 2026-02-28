# Savaşan İHA: Hedef Takip, Dinamik Yol Planlama ve HSS Kaçınma - Literatür ve Mimari Raporu

**Tarih:** 2026-02-16  
**Konu:** Teknofest Savaşan İHA yarışması için hedef takip algoritması, dinamik waypoint takibi ve HSS kaçınma mimarisi

---

## 1. Dinamik Waypoint ve HSS Kaçınma Literatürü

Geleneksel otopilotlar statik rotalar üzerinden uçar. Ancak yarışma sahasında anlık beliren HSS alanlarından kaçınmak ve dinamik sözlükteki koordinatları takip etmek için "Lokal Yol Planlama" (Local Path Planning) algoritmalarına ihtiyaç vardır.

### Seçenek A: Yapay Potansiyel Alanlar (APF) [Önerilen]
- **Mantık:** Hedef nokta çekim kuvveti, HSS alanları itki kuvveti oluşturur. İHA bileşke yönünde uçar.
- **Avantaj:** Hesaplaması hafif (100Hz+), HSS'nin itici gücü kademeli artırılabilir.

### Seçenek B: VFH (Vector Field Histogram)
- **Mantık:** HSS koordinatlarını histograma dönüştürür, engelsiz yönleri bulur.
- **Avantaj:** Birden fazla engelde iyi çalışır.

### Seçenek C: RRT* (Rapidly-exploring Random Tree Star)
- **Mantık:** Rastgele ağaçlarla en kısa engelsiz yol bulunur.
- **Dezavantaj:** Sabit kanatlı dönüş yarıçapı kısıtları ile uyumsuz.

---

## 2. Hedef Takip Algoritması (Görüntü + GPS Füzyonu)

4 saniye boyunca hedefi merkezde tutmak için YOLO bounding box + rakip GPS verisi birleştirilmelidir.

### Görsel Servo Kontrolü (Visual Servoing) [Önerilen]
- **IBVS (Image-Based Visual Servo):** Rakip kameranın sağındaysa → Yaw/Roll düzeltmesi
- Bounding box merkezi ile görüntü merkezi arasındaki fark → PID kontrolcü → açısal komut

### Genişletilmiş Kalman Filtresi (EKF) ile Tahmin
- Rakip kameradan çıksa bile GPS hız/konum verisi → 1s sonraki tahmin
- YOLO kaçırsa bile takip devam eder

### Kilitlenme (Lock-on) Mantığı
- YOLO hedefi tespit → 4 saniye timer başla
- 4 saniye boyunca hedef kameranın ±%20 merkezinde kalmalı
- Timer tamamlanırsa → "Kilitlenme Başarılı" → sıradaki göreve geç

---

## 3. Çoklu İHA Simülasyonu (Gazebo + PX4 SITL)

### PX4 Multi-Vehicle
```bash
cd ~/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2 -m plane
```

### MAVROS Bağlantıları
- UAV1 (bizim): port 14540 → `namespace:=uav1`
- UAV2 (rakip): port 14541 → `namespace:=uav2`
- `/uav2/mavros/global_position/global` → rakip konum

### QGroundControl
- Birden fazla aracı otomatik tanır (Vehicle 1, Vehicle 2)

---

## 4. Önerilen State Machine Mimarisi

| State | Açıklama | Geçiş Koşulu |
|-------|----------|---------------|
| `TAKEOFF` | Güvenli irtifaya çık | İrtifaya ulaşıldı → PATROL |
| `PATROL` | Waypoint sözlüğünü döngüsel takip | Hedef tespit → TRACKING, HSS aktif → EVADE |
| `EVADE` | HSS'den APF/teğet ile kaçış | HSS kapandı → PATROL |
| `TRACKING` | YOLO + GPS ile 4s hedef kilitleme | 4s tamamlandı → PATROL |
| `LAND` | Manuel komut ile Home'a iniş | İniş tamamlandı → END |

### Geçiş Diyagramı
```
TAKEOFF → PATROL ⇄ EVADE
                 ⇄ TRACKING
         PATROL → LAND (manuel komut)
```

---

## 5. Teknik Bileşenler

| Bileşen | ROS2 Topic/Service | Açıklama |
|---------|-------------------|----------|
| YOLO Tespit | `/yolo/detections` | Bounding box (x, y, w, h, class) |
| Rakip GPS | `/uav2/mavros/global_position/global` | Rakip pozisyon |
| Kendi GPS | `/mavros/global_position/global` | Kendi pozisyon |
| Setpoint | `/mavros/setpoint_raw/local` | Konum+hız kontrolü |
| Attitude | `/mavros/setpoint_raw/attitude` | Yaw/pitch kontrolü |
| Mod | `/mavros/set_mode` | OFFBOARD, AUTO.LAND |

---

## 6. Sonuç

HSS kaçınma + hedef takip + dinamik waypoint sistemi için önerilen mimari:
- **State Machine** tabanlı görev yönetimi
- **APF** ile HSS kaçınma (basit, hızlı)
- **IBVS** ile görsel servo (YOLO → Yaw/Pitch PID)
- **EKF** ile rakip konum tahmini (opsiyonel)
- **Dinamik sözlük** ile waypoint müdahalesi
