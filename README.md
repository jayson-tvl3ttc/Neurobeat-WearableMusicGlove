# Neurobeat Wearable Music Glove  
**A Wearable Glove Controller for Interactive Music Performance**

---

## Overview

**MusicGlove** is an interactive wearable instrument that transforms hand gestures into musical control signals.  
Built using **Arduino Nano 33 BLE**, **flex sensors**, and an **IMU (Inertial Measurement Unit)**, it connects to **Max/MSP** via serial communication to control a custom drum machine in real time.

---

##  Glove Prototype

![integration](https://github.com/user-attachments/assets/66b6a5bf-b3e4-4919-9167-c1de7d79ee12)

*Figure 1. The physical MusicGlove prototype with flex sensors and LED feedback.*

---

## System Overview

<img width="2988" height="1837" alt="music glove (6)" src="https://github.com/user-attachments/assets/aa9b0f38-1038-4e14-9a24-180490e7ce79" />


##  Features

-  **Real-time Control** – Converts finger motion and wrist movement into sound parameters.  
-  **Five Flex Sensors** – Map finger bending to different drum triggers.  
-  **IMU-Based Motion Tracking** – Detects tilt and rotation to modulate sound synthesis.  
-  **LED Feedback** – Provides visual cues for calibration and rhythm visualization.  
-  **Power Design** – Operates entirely via serial USB.  
-  **Max/MSP Integration** – Translates sensor data into musical events and controls a drum machine patch.

---

##  Technical Implementation

| Module | Description |
|---------|--------------|
| **Microcontroller** | Arduino Nano 33 BLE |
| **Sensors** | 5× Flex sensors, 1× Integrated IMU |
| **Software** | Arduino IDE, Max/MSP 8 |
| **Communication** | Serial at 9600 baud |
| **Audio Engine** | Max/MSP drum machine patch |
| **LED Feedback** | LED indicators for user feedback |
| **Power Supply** | USB via serial |

<img width="1021" height="498" alt="music glove (7)" src="https://github.com/user-attachments/assets/96de9668-a213-427a-a9e8-d9fed1bafaef" />

---

##  Max/MSP Integration

The Max patch parses incoming serial data from Arduino and maps it to:
- Drum synthesis parameters (kick, snare, hi-hat, tom, cowbell)  
- Rhythm randomization controls  
- Visual LED triggers and MIDI output  
- Gesture-based modulation (tilt ↔ reverb, flex ↔ pitch)
<img width="3918" height="1408" alt="music glove (8)" src="https://github.com/user-attachments/assets/f72215b4-5a62-45d9-a9cf-7b02af590070" />

---

## Folder Structure
Arduino/   → Arduino code and calibration scripts  
MaxMSP/    → Max patches and UI elements  
Docs/      → Circuit diagrams, demo video and user guide 

---
## Future Development

-  BLE MIDI communication for wireless control  
-  Gesture recognition via machine learning models  
-  Integration with Ableton Live through Max for Live  
-  Educational kit version for workshops and outreach  

---

##  Author

**Jayson Chen**  
Master’s in Audio and Music Technology  
University of York  

Contact: *[jaysonchen816@gmail.com]*  



