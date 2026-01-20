# ESP32-CAM Edge Impulse Anomaly Detection

![ESP32](https://img.shields.io/badge/ESP32-blue?logo=espressif)
![Edge Impulse](https://img.shields.io/badge/Edge%20Impulse-blueviolet?logo=edgeimpulse)
![Computer Vision](https://img.shields.io/badge/Computer%20Vision-Anomaly%20Detection-red)
![Deep Learning](https://img.shields.io/badge/Deep%20Learning-TinyML-orange)
![IoT](https://img.shields.io/badge/IoT-green?logo=iot)

This project demonstrates **real-time visual anomaly detection** using an **ESP32-CAM (AI Thinker)** module and a machine learning model trained with **Edge Impulse**.

The system learns what is considered *normal* from training images and detects **anomalous regions** in live camera frames. Detected anomalies are visualized as **red markers** on a live video stream served over Wi-Fi.

<p float="left">
  <img src="Example.gif" />
  <img src="Non-Anomalous Sample.jpg" width="240" alt="test"/>
</p>

## Contents
- [Features](#features)
- [How It Works](#how-it-works)
- [Normal vs Anomaly](#normal-vs-anomaly)
- [Hardware Requirements](#hardware-requirements)
- [Known Limitations](#known-limitations)
- [Installation](#installation)
- [License](#license)
- [Need help](#need-help)

## Features
- Real-time camera capture using ESP32-CAM
- Visual Anomaly Detection model trained with Edge Impulse
- Detection of abnormal regions (unsupervised learning)
- Anomalies displayed as red dots on the video stream
- Live MJPEG stream via built-in HTTP web server
- Runs fully on-device (no cloud inference)

## How It Works
1) ESP32-CAM captures a frame in RGB565 format
2) Image is converted to RGB888 and resized
3) Frame is passed to the Edge Impulse model
4) Model outputs anomaly score and visual anomaly regions
5) Detected anomalies are:
   - Stored with coordinates
   - Drawn as red filled dots on the frame
6) Live annotated video is streamed via browser

## Normal vs Anomaly
- Normal images: Used during training (baseline appearance)
- Anomaly: Any visual deviation from learned normal patterns

Examples:
- Missing object
- Extra object
- Position change
- Surface damage
- Unexpected movement

## Hardware Requirements
- ESP32-CAM (AI Thinker)
- USB-to-TTL adapter (FTDI, CP2102, etc.)
- Jumper wires

## Known Limitations
- Sensitive to lighting and background changes
- Camera position must remain fixed
- Lower resolution may miss small anomalies
- Higher resolutions reduce FPS
- Many anomaly detections increase RAM usage
- ESP32-CAM has limited compute and memory resources
- Detects anomaly regions, not anomaly types

## Installation

### Getting Ready
1) Open Arduino IDE -> File -> Preferences
2) Add following URL to **Additional boards manager URLs**;
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

### Collecting Data to Train the AI Model
1) Complete the [Getting Ready](#getting-ready) steps
2) Open Arduino IDE -> Library Manager
3) Install the `EloquentEsp32Cam` library
4) Go File -> Examples -> EloquentEsp32Cam -> `Collect_Images_for_EdgeImpulse`
5) Upload the code to tje ESP32-CAM (For programming, see [Programming ESP32-CAM](#programming-esp32-cam))
6) Collect **only normal images** of the object or scene (collect at least 200 images)
**Data collection guidelines:**
- Capture images from a **fixed camera position**
- Keep the **background consistent**
- Avoid anomalies during data collection
- Slight variations in lighting are acceptable, but avoid extreme changes
- Ensure the object appears **centered and fully visible**
- Do not include damaged, missing, or abnormal cases in training data

These images define what the model considers *normal*.

Any deviation from this learned pattern will be detected as an anomaly during inference.

### Programming ESP32-CAM
The ESP32-CAM does not include an integrated USB-to-serial programmer so an external FTDI (USB-to-TTL) programmer is used for uploading code. (You can also use an Arduino Uno as a USB-to-serial adapter. For more information, refer to tutorial videos on YouTube.)
<p float="left">
  <img src="FTDI Programmer Pinout.png" width="500" />
</p>

### Train an AI model with Edge Impulse
1) Go to **https://studio.edgeimpulse.com/**
2) Create new project
3) Navigate to **Data acquisition -> Add data -> Upload data**
4) Select the folder containing only normal images
7) Click on upload data button
8) Go **Labelling Queue** and label your images
9) Navigate to **Create Impulse**
10) Add an **Image Processing** block
11) Add an **Visual Anomaly Detection** learning block
12) Save the impulse
13) Go to **Image** and set **Color depth** to **RGB**
14) Save parameters and click **Generate Features**
15) Go to **Visual Anomaly Detection**
16) Click **Start Training**
17) After training is complete, go to **Deployment**
18) Select **Arduino Library** as the deployment option
19) Set the target to **Espressif ESP-EYE (ESP32 240MHz)**
20) Click **Build** and download the generated library
21) In Arduino IDE, go to Sketch → Include Library → Add .ZIP Library
22) Open [AnomalyDetection.ino](https://github.com/Mali03/ESP32-CAM-AnomalyDetection/blob/main/AnomalyDetection.ino) and replace
```cpp
#include <Anomaly_Detection_inferencing.h>
```
with your own model header file

Update the WiFi credentials in the code before uploading it to the ESP32-CAM:
```cpp
const char *ssid = "*******";
const char *password = "*******";
```
23) Upload the code and open the Serial Monitor at **115200 baud** and copy the ip address
25) Open a browser and navigate to:
```
http://ESP32_IP_ADDRESS/
```
You will see the live camera stream with detected objects highlighted.

## License
This project is licensed under the **MIT License** - see the [LICENSE](https://github.com/Mali03/ESP32-CAM-AnomalyDetection/blob/main/LICENSE) file for details.

## Need Help
If you need any help contact me on [LinkedIn](https://www.linkedin.com/in/mali03/).

⭐ If you like this project, don’t forget to give it a star on GitHub!
