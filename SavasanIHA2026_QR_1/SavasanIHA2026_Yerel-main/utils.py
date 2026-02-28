# utils.py: Sistemin arka planda ihtiyaç duyduğu yardımcı fonksiyonları tutar. Özellikle WeChat QR 
# algoritmasının çalışması için gereken Caffe yapay zeka modellerini internetten (GitHub üzerinden) 
# otomatik indirmeye yarayan kodlar ve Jetson (Nvidia) donanımlarında kamerayı optimize kullanmak için gerekli 
# GStreamer ardışık düzeni (pipeline) burada yer alır.


import os
import requests

def check_and_download_models(target_dir="."):
    """
    Checks for WeChatQRCode model files and downloads them if missing.
    Returns True if all files are present (or downloaded successfully), False otherwise.
    """
    base_url = "https://github.com/WeChatCV/opencv_3rdparty/raw/wechat_qrcode_model/"
    files = [
        "detect.prototxt",
        "detect.caffemodel",
        "sr.prototxt",
        "sr.caffemodel"
    ]

    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    all_present = True
    for fname in files:
        fpath = os.path.join(target_dir, fname)
        if not os.path.exists(fpath):
            print(f"📥 Downloading missing model: {fname}...")
            try:
                url = base_url + fname
                response = requests.get(url, stream=True)
                if response.status_code == 200:
                    with open(fpath, 'wb') as f:
                        for chunk in response.iter_content(chunk_size=8192):
                            f.write(chunk)
                    print(f"✅ Downloaded {fname}")
                else:
                    print(f"❌ Failed to download {fname}: Status {response.status_code}")
                    all_present = False
            except Exception as e:
                print(f"❌ Error downloading {fname}: {e}")
                all_present = False
        else:
            # print(f"✅ Found {fname}")
            pass

    return all_present

def get_gstreamer_pipeline(capture_width=1280, capture_height=720, framerate=60):
    """
    Generates a GStreamer pipeline string for Jetson ISP usage (nvarguscamerasrc).
    Output is BGR for OpenCV compatibility.
    """
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=1"
    )
