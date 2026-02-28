# model_indirme.py: Sistemi ilk başta kurarken gereken yapay zeka modellerini indirmeye yarayan bağımsız bir script dosyasıdır. 
# (İçeriği büyük ihtimalle utils.py ile benzerlik gösterir).



import urllib.request
import os

print("🔄 WeChat QR Modelleri için DOĞRU linklere bağlanılıyor...")

# Doğru GitHub RAW linkleri (wechat_qrcode dalından)
dosyalar = {
    "detect.caffemodel": "https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/wechat_qrcode/detect.caffemodel",
    "detect.prototxt": "https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/wechat_qrcode/detect.prototxt",
    "sr.caffemodel": "https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/wechat_qrcode/sr.caffemodel",
    "sr.prototxt": "https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/wechat_qrcode/sr.prototxt"
}

basarili_sayisi = 0

for dosya_adi, url in dosyalar.items():
    print(f"⬇️ İndiriliyor: {dosya_adi} ...")
    try:
        # Dosyayı indir ve kaydet
        urllib.request.urlretrieve(url, dosya_adi)

        # Dosya gerçekten indi mi kontrol et (Boyutu 0 olmamalı)
        if os.path.getsize(dosya_adi) > 1000:
            print(f"✅ BAŞARILI: {dosya_adi} indi.")
            basarili_sayisi += 1
        else:
            print(f"❌ HATA: {dosya_adi} boş indi (0 byte). Link engellenmiş olabilir.")

    except Exception as e:
        print(f"❌ HATA: {dosya_adi} indirilemedi. Sebep: {e}")

print(f"\nSonuç: 4 dosyadan {basarili_sayisi} tanesi hazır.")

if basarili_sayisi == 4:
    print("🚀 Harika! Şimdi main.py kodunu tekrar çalıştırabilirsin.")
else:
    print("⚠️ Hâlâ sorun varsa, internetin GitHub'ın 'raw' sunucularını engelliyor olabilir.")