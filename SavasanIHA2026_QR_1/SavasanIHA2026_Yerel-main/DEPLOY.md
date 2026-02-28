# Jetson Orin NX Deployment Guide

## 1. Environment Setup

On your Jetson Orin NX, ensure you have JetPack installed with CUDA and TensorRT support.

Install dependencies:
```bash
pip install ultralytics opencv-python numpy
```

Note: On Jetson, you usually use the built-in OpenCV (with CUDA/GStreamer). If you install `opencv-python` via pip, it might not have hardware acceleration. Ensure your OpenCV is CUDA-enabled.

## 2. TensorRT Model Export

To achieve maximum FPS, you **must** export your YOLO model to a TensorRT engine.

Run this command on the Jetson:

```python
from ultralytics import YOLO

# Load your custom trained model
model = YOLO("best.pt")

# Export to TensorRT Engine (FP16)
# device=0 refers to the GPU
model.export(format="engine", half=True, device=0)
```

This will create `best.engine`.

## 3. Running the System

Update `main.py` to use the engine file:

```python
MODEL_PATH = "best.engine"
```

Run the system:

```bash
python main.py
```

## 4. Hardware Optimizations

### Camera
Ensure your Global Shutter camera is set to a high shutter speed (low exposure time) to minimize motion blur.
Adjust gain (ISO) to compensate for brightness.

### Power Mode
Set Jetson to MAX power mode:
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## 5. Troubleshooting

- **"AttributeError: module 'cv2' has no attribute 'wechat_qrcode_WeChatQRCode'"**:
  Ensure you have `opencv-contrib-python` installed, or build OpenCV from source with `opencv_contrib` modules enabled.

- **Low FPS**:
  - Check if `model.export` was successful and `best.engine` is being used.
  - Ensure `AsyncQRReader` is running in a separate thread (it is by default in this code).
