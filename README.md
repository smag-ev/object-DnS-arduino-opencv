# 🎯 Object Tracking & Servo Control

A compact computer-vision and IoT project that combines **OpenCV object tracking**, **Arduino servo control**, **A* path planning**, and a **Processing-based radar visualization**.

The Python application lets the user select an object from a live camera feed, tracks it with OpenCV, estimates a stable lock state, and maps the horizontal position to a servo angle.

## ✨ Features

- 🎥 Live camera/IP-camera input
- 🎯 Manual ROI target selection
- 🧠 CSRT/KCF tracking
- 🔒 Stability-based lock detection
- 🔄 Smoothed servo positioning
- 🧭 Servo inversion and configurable limits
- 🗺️ A* grid-based path-planning demonstration
- 📡 Serial communication with Arduino
- 📊 CSV event logging
- 🔊 Optional tracking/lock sound feedback
- 🖥️ Processing radar visualization

## 📁 Project Structure

```text
.
├── main.py
├── hardware.ino
├── radar.pde
├── requirements.txt
├── .gitignore
└── README.md
```

Generated runtime files such as the CSV log are intentionally ignored by Git.

## 🛠️ Requirements

### Python

- Python 3.9+
- OpenCV
- OpenCV Contrib
- PySerial

Install the dependencies:

```bash
pip install -r requirements.txt
```

### Arduino

- Arduino IDE
- Servo library

### Processing

- Processing 4.x
- Processing Serial library

## ⚙️ Configuration

Update the configuration at the top of `main.py`:

```python
CAMERA_URL = "URL HERE"
SERIAL_PORT = "COM9"
BAUD_RATE = 115200
```

Useful tracking parameters:

```python
TRACKER_TYPE = "CSRT"
SERVO_MIN = 30
SERVO_MAX = 150
REST_ANGLE = 90
LOCK_FRAMES = 15
```

For the Processing visualization:

```java
String SERIAL_PORT = "COM9";
int BAUD_RATE = 9600;
```

## 🚀 Running

### 1. Upload the Arduino sketch

Open `hardware.ino` in Arduino IDE, select the correct board and serial port, and upload it.

### 2. Start the Python application

```bash
python main.py
```

Select the target when the ROI selector appears, then press **Enter** or **Space**.

### 3. Start the radar visualization

Open `radar.pde` in Processing and configure its serial port before running it.

## 🎮 Controls

| Key | Action |
|---|---|
| `N` | Reset and select a new target |
| `R` | Reset servo and tracking state |
| `I` | Invert servo direction |
| `Q` | Quit |

## 🔌 Serial Protocol

The Arduino tracking controller accepts:

| Command | Purpose |
|---|---|
| `S<angle>` | Set the tracking-servo target angle |
| `R<angle>` | Reset the tracking servo |
| `STOP` | Hold the current position |

Example:

```text
S90
R90
STOP
```

The Processing visualization expects angle/range messages containing two numeric values, for example:

```text
90 120
```

## 🧠 Architecture

```text
             Camera
                │
                ▼
        ┌─────────────────┐
        │  OpenCV Tracker │
        └────────┬────────┘
                 │
          Target Position
                 │
        ┌────────┴────────┐
        ▼                 ▼
  Lock Detection       A* Planner
        │                 │
        └────────┬────────┘
                 ▼
          Servo Mapping
                 │
                 ▼
              Serial
                 │
                 ▼
             Arduino
                 │
                 ▼
          Tracking Servo


      Radar / Range Source
                 │
                 ▼
          Serial Stream
                 │
                 ▼
        Processing Radar UI
```

## 🧪 Troubleshooting

### OpenCV tracker is unavailable

Install the contrib package:

```bash
pip install opencv-contrib-python
```

### Serial port cannot be opened

Check that:

- The Arduino is connected.
- `SERIAL_PORT` is correct.
- Arduino Serial Monitor is closed.
- Another application is not using the same port.

### Camera feed does not open

Check `CAMERA_URL`.

For a local webcam, OpenCV can use a camera index:

```python
CAMERA_URL = 0
```

### Sound notifications do not play

Sound feedback uses Windows `winsound`. The application continues without sound on platforms where `winsound` is unavailable or the configured WAV file is missing.

## 📝 Implementation Notes

The Python application uses **manual target selection followed by visual tracking**. It is therefore more accurately described as an object-tracking system than a neural-network object detector.

The A* component is a grid-based planning demonstration operating on the display coordinate space.

The Processing sketch is a separate visualization component and should use a serial data source appropriate to the connected range sensor.

## 📜 License

Add an appropriate open-source license before publishing the repository if your project permits redistribution.


## 🔒 Physical Actuation Placeholder

The original project contains a physical actuation path. In this GitHub-ready
copy, the two physical invocation points have been replaced with clearly marked
placeholders. The surrounding project structure, tracking, locking, servo
control, serial architecture, logging, planner, and radar visualization remain
intact.
