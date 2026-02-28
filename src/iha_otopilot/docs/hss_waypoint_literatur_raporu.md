# HSS Yasaklı Bölge Kaçınma ve Dinamik Waypoint Takibi - Literatür Raporu

**Tarih:** 2026-02-16  
**Konu:** İHA Otonom Uçuşunda Dinamik Yasaklı Bölge Kaçınma ve Manuel Müdahale Yöntemleri

---

## 1. Giriş ve Problem Tanımı

Teknofest Savaşan İHA yarışmasında, İHA'nın belirli bir bölgede otonom waypoint takibi yaparken **Hava Savunma Sistemi (HSS)** tarafından belirlenen yasaklı bölgelerden kaçınması gerekmektedir. HSS bölgeleri:
- Hakem tarafından anlık aktive edilir
- 10 saniye gecikme ile aktif olur
- 1 dakika aktif kalır, sonra kapanır
- İHA bu bölgelere girmemelidir

Bu problem, literatürde **"dynamic geofencing"** ve **"keep-out zone avoidance"** olarak bilinmektedir.

---

## 2. Yaklaşım Seçenekleri

### Seçenek A: Point-in-Polygon (PIP) + Güvenli Waypoint Kaçış

**Açıklama:** Yasaklı bölge çokgen (polygon) olarak tanımlanır. Her kontrol döngüsünde uçağın pozisyonu ve hedef waypoint'i PIP algoritması ile kontrol edilir.

**Algoritma:**
1. Ray Casting (Işın Atma) algoritması ile nokta-çokgen testi
2. Hedef waypoint yasaklı alanda ise → en yakın güvenli waypoint'e yönlen
3. Bölge kapandığında → orijinal waypoint takibine dön

**Avantajları:**
- Basit implementasyon
- Hesaplama maliyeti düşük
- Gerçek zamanlı kontrol mümkün

**Dezavantajları:**
- Sadece 2D kontrol (irtifa hariç)
- Karmaşık çokgenler için yavaşlayabilir

**Kaynaklar:**
- IEEE: "Geofencing for Unmanned Aerial Vehicle Management" (PIP + safety buffer)
- AIAA: "Real-Time Geofencing Algorithm for UAVs"

---

### Seçenek B: Artificial Potential Field (Yapay Potansiyel Alan)

**Açıklama:** Yasaklı bölgeler "itici alan" (repulsive field), waypoint'ler "çekici alan" (attractive field) olarak tanımlanır. İHA, toplam potansiyel alanın gradyanına göre hareket eder.

**Algoritma:**
1. Waypoint → çekici kuvvet (attractive force)
2. HSS bölgesi → itici kuvvet (repulsive force)
3. Toplam kuvvet → uçuş yönü ve hızı

**Avantajları:**
- Dinamik engel kaçınmaya uygun
- Yumuşak geçişler
- Çoklu yasaklı bölge desteği

**Dezavantajları:**
- Lokal minimum problemi (iki itici alan arasında sıkışma)
- Fixed-wing için dönüş yarıçapı kısıtları ile uyumsuz olabilir

**Kaynaklar:**
- MDPI Drones: "Path Planning and Obstacle Avoidance"
- ResearchGate: "APF-based UAV Path Planning with Dynamic Obstacles"

---

### Seçenek C: Tangent Line Escape (Teğet Çizgi Kaçış)

**Açıklama:** Yasaklı bölgeye yaklaşıldığında, bölge sınırına teğet bir çizgi hesaplanır ve uçak bu teğet boyunca bölgeyi dolanır.

**Algoritma:**
1. Yasaklı bölge merkezi ve yarıçapı belirlenir (veya poligon köşeleri)
2. Uçağın mevcut konumundan bölgeye teğet çizgiler hesaplanır
3. En kısa yolu veren teğet seçilir
4. Uçak teğet boyunca ilerler, bölgeyi geçtikten sonra orijinal rotasına döner

**Avantajları:**
- Fixed-wing için dönüş yarıçapına uygun
- Geometrik olarak en kısa kaçış yolu
- Öngörülebilir davranış

**Dezavantajları:**
- Çoklu bölgeler arasında karmaşık hesaplama
- Ani HSS aktivasyonunda tepki süresi yetersiz kalabilir

**Kaynaklar:**
- ArXiv: "Tangent Intersection Guidance for UAV Path Planning"
- ResearchGate: "Heuristic-Based Tangent Graph Path Planning"

---

### Seçenek D: Dinamik Waypoint Listesi + PIP Hibrit Yaklaşım (ÖNERİLEN)

**Açıklama:** Bu yaklaşım, mevcut kodumuzun mimarisine en uygun olanıdır. Basit ve etkili bir hibrit yöntem:

**Mimari:**
1. **Dinamik Waypoint Sözlüğü:** Operatör tarafından anlık güncellenen waypoint listesi
2. **HSS Bölge Yöneticisi:** Yasaklı bölgeleri zamanlayıcı ile yöneten sınıf
3. **PIP Kontrol Katmanı:** Her döngüde hedef waypoint'in yasaklı bölgede olup olmadığını kontrol
4. **Güvenli Waypoint Seçici:** Yasaklı bölge aktifken alternatif waypoint belirler

**Neden Bu Seçenek?**
- Mevcut state machine mimarimize (sim1.py, qr2.py) tam uyumlu
- OFFBOARD modda çalışır
- Manuel müdahaleye açık (waypoint sözlüğü runtime'da değiştirilebilir)
- Teknofest kurallarına uygun (10s gecikme, 1dk aktiflik)

---

## 3. Teknofest Savaşan İHA Kuralları ile İlişki

| Kural | İmplementasyon |
|-------|---------------|
| Otonom kalkış | AUTO.TAKEOFF + ARM |
| Waypoint takibi | OFFBOARD + position setpoint |
| HSS yasaklı bölge | PIP + zamanlayıcı (10s gecikme, 60s aktif) |
| Manuel müdahale | Dinamik waypoint sözlüğü + LAND komutu |
| Eve dönüş | AUTO.LAND komutu |

---

## 4. PX4/MAVROS Teknik Detaylar

### Geofencing Mekanizması
- PX4, dahili olarak inclusion/exclusion geofence destekler
- QGroundControl üzerinden çokgen/daire bölgeler tanımlanabilir
- **Ancak:** Dinamik (runtime'da değişen) geofence desteği sınırlıdır
- **Çözüm:** Companion computer (bizim ROS2 node'umuz) üzerinde yazılımsal geofence

### OFFBOARD Mode
- Minimum 2Hz setpoint stream gerektirir (bizim: 20Hz / 50ms)
- Position, velocity, attitude setpoint destekler
- Companion computer tam kontrol sağlar

### Mod Geçişleri
- `OFFBOARD` → waypoint takibi
- `AUTO.LAND` → home konumuna iniş
- `/mavros/set_mode` servisi ile geçiş

---

## 5. Benzer Çalışmalar

### Akademik
1. **"Dynamic Geofencing for UAV Fleet Management"** (IEEE 2024) - Sürü İHA'lar için dinamik geofence yönetimi
2. **"Real-Time Path Replanning for Fixed-Wing UAVs"** (MDPI 2023) - RRT* + DWA hibrit yaklaşım
3. **"PX4-Avoidance"** (GitHub/PX4) - Açık kaynak engel kaçınma framework'ü

### Endüstriyel
1. **DJI Geofence System** - Statik no-fly zone veritabanı (dinamik değil)
2. **AirMap** - UTM (UAS Traffic Management) geofencing API
3. **PX4 Failsafe Geofence** - Silindirik sınır + failsafe aksiyon

### Teknofest Ekosistemi
- TÜBİTAK destekli takımlar genellikle:
  - PX4 + MAVROS + ROS2 mimarisi kullanır
  - State machine tabanlı görev yönetimi yapar
  - OFFBOARD modda waypoint takibi yapar
  - Yazılımsal geofence implementasyonu tercih eder (PX4 dahili geofence yerine)

---

## 6. Önerilen Yaklaşım Detayı

### Mimari Diyagram

```
┌─────────────────────────────────────────────────┐
│                OPERATÖR                          │
│  - Waypoint güncelle                            │
│  - HSS bölgesi aktive et                        │
│  - LAND komutu gönder                           │
└──────────┬──────────────────────┬───────────────┘
           │                      │
     Waypoint Dict           HSS Komutu
           │                      │
┌──────────▼──────────────────────▼───────────────┐
│          hss_waypoint_mission.py                 │
│                                                  │
│  ┌──────────┐  ┌──────────┐  ┌────────────┐    │
│  │ Waypoint  │  │   HSS    │  │   PIP      │    │
│  │ Manager   │  │ Manager  │  │  Checker   │    │
│  └─────┬────┘  └─────┬────┘  └─────┬──────┘    │
│        │              │              │           │
│        └──────────┬───┘──────────────┘           │
│                   │                              │
│           Control Loop (50ms)                    │
│        TAKEOFF → PATROL → LAND                   │
└──────────────────┬───────────────────────────────┘
                   │
            MAVROS / PX4
```

### Algoritma Akışı

1. **TAKEOFF** → İstenen irtifaya çık
2. **PATROL** → Waypoint sözlüğünden sıradaki hedefe git
3. **HSS Aktive** → 10s timer başlat
4. **HSS Aktif** → PIP kontrolü yap
   - Hedef waypoint yasaklı alanda mı?
   - Evet → Güvenli waypoint'e yönlen (bölge sınırı dışında en yakın nokta)
   - Hayır → Normal waypoint takibine devam
5. **HSS Kapandı** (60s sonra) → Normal waypoint takibine dön
6. **LAND Komutu** → AUTO.LAND ile home'a in

---

## 7. Sonuç ve Değerlendirme

| Kriter | Seçenek A (PIP) | Seçenek B (APF) | Seçenek C (Tangent) | Seçenek D (Hibrit) |
|--------|:---:|:---:|:---:|:---:|
| Implementasyon kolaylığı | ★★★★ | ★★ | ★★★ | ★★★★★ |
| Fixed-wing uyumluluğu | ★★★ | ★★ | ★★★★ | ★★★★ |
| Mevcut kod uyumu | ★★★★ | ★★ | ★★★ | ★★★★★ |
| Teknofest kuralları | ★★★★ | ★★★ | ★★★ | ★★★★★ |
| Gerçek zamanlılık | ★★★★★ | ★★★ | ★★★★ | ★★★★★ |

**Önerilen:** Seçenek D (Dinamik Waypoint + PIP Hibrit) - Mevcut kod tabanımıza en uygun, basit ve etkili çözüm.
