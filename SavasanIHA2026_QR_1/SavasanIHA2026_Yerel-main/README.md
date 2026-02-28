# Combat UAV - QR Code Reading and Target Tracking System

## Project Overview

This project combines YOLOv8 object detection and QR code reading technologies to create a UAV (Unmanned Aerial Vehicle) simulation system. The system detects targets from video streams, tracks them, and reads QR codes on the targets.

## Features

- **YOLOv8-Based Object Detection**: Target detection with custom trained model
- **QR Code Reading**: Automatic QR code reading on detected targets
- **Target Locking**: Target tracking and locking system based on center crosshair
- **Visual Feedback**: 
  - Red box: Target tracking (not locked)
  - Green box: Target locked
  - Center crosshair
  - Target-crosshair connection line

## Requirements
```bash
pip install opencv-python
pip install numpy
pip install ultralytics
pip install pyzbar
```

### Additional Requirements (for pyzbar)

**Windows:**
```bash
pip install pyzbar[scripts]
```

**Linux:**
```bash
sudo apt-get install libzbar0
```

**macOS:**
```bash
brew install zbar
```

## File Structure
```
project/
│
├── main.py                 # Main program
├── best.pt                 # Trained YOLOv8 model
├── QRTest_video1.mp4      # Test video
└── README.md              # This file
```

## Usage

### 1. Model Training (Google Colab)
```python
# YOLOv8 model training in Google Colab
from ultralytics import YOLO

# Create model
model = YOLO('yolov8n.pt')  # or yolov8s.pt, yolov8m.pt

# Training
results = model.train(
    data='dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16
)

# Save model
model.save('best.pt')
```

### 2. Running the Program
```bash
python main.py
```

### 3. Controls

- **Q key**: Exit the program
- Window opens and adjusts automatically

## Configuration

You can edit the settings at the beginning of the code:
```python
video_dosya_adi = "QRTest_video1.mp4"  # Video file
model_dosya_adi = "best.pt"             # Model file
KILIT_MESAFESI = 150                    # Lock sensitivity (pixels)
```

## System Logic

1. **Video Loading**: Specified video file is opened
2. **Model Loading**: YOLOv8 model is loaded into memory
3. **Frame Processing**: For each frame:
   - Object detection is performed
   - QR codes are searched within detected objects
   - Distance to target's center point is calculated
   - Target lock status is determined
4. **Visualization**: Results are displayed on screen

## Color Codes

- **Blue**: Center crosshair
- **Red**: Target tracking (not locked)
- **Green**: Target locked
- **Yellow**: QR code text

## Notes

- Model file (`best.pt`) must be in the project root directory
- Video file must be in supported formats (mp4, avi, etc.)
- Target must be clear enough for QR code reading
- System works in continuous reading mode (QR codes are searched in all frames)

## Troubleshooting

**Video not found error:**
- Ensure the video file is in the correct location
- Verify that the filename matches the one specified in the code

**Model cannot be loaded:**
- Check that the `best.pt` file is in the correct location
- Ensure the model is in YOLOv8 format

**QR code not reading:**
- Ensure the QR code video quality is sufficient
- Make sure the target is not moving too fast
- Check lighting conditions

## Developer Notes

- Model was trained on Google Colab
- Code was written and tested in local environment
- System can also work with real-time video streams (by changing the video source)

## License

This project was developed for educational purposes.

---

**Note:** This system is for simulation purposes. Real UAV applications require additional security measures and legal permissions.