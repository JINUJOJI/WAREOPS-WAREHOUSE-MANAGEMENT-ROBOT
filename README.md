# WAREOPS: Warehouse Management Robot

WAREOPS is a line-following, pick-and-place warehouse management robot developed as part of an engineering project at VIT-AP University.  
It automates item transport and placement within a warehouse using sensors, a forklift mechanism, and wireless communication between microcontrollers.

---

## 💡 Project Objective

To optimize warehouse operations by automating the movement, scanning, and placement of packages using a robot equipped with:

- **Infrared sensors** for line navigation
- **Ultrasonic sensors** for obstacle and height detection
- **QR code-based routing** at intersections
- **Wi-Fi modules** for device-to-device communication
- **ESP32-CAM** for QR scanning and object size classification

---

## 🔧 Features

- **Line Following Navigation**  
  IR sensors track a predefined path across warehouse grid lines.

- **Pick and Place Mechanism**  
  A motorized forklift lifts and places packages at specific rack heights.

- **Obstacle Detection**  
  Dual ultrasonic sensors detect objects in front and measure lift height.

- **QR Code-Based Routing**  
  At intersections, an ESP32-CAM scans QR codes to decide the next route.

- **Camera-Based Object Size Detection**  
  ESP32-CAM detects and classifies objects as Small (S), Medium (M), or Large (L).

- **Wireless Communication**  
  Arduino and ESP32-CAM communicate over Serial and Wi-Fi for control and data transfer.

---

## 🛠️ Hardware Used

- **Arduino Uno** – Main navigation & lift control  
- **ESP32-CAM (AI Thinker)** – QR scanning & object classification  
- **ESP8266** – Rack communication module  
- **L298 Motor Driver** – Drive motors  
- **Lift Motor Driver** – Forklift control  
- **DC Motors** – Robot movement  
- **IR Sensors** – Line following  
- **Ultrasonic Sensors** – Obstacle detection & lift height sensing  
- **Jumper Wires, Breadboards, Battery Pack**  

---

## 💻 Software Stack

- **Arduino IDE** – Programming Arduino & ESP32-CAM  
- **C/C++ (Arduino)** – Microcontroller logic  
- **HTML + JavaScript** – ESP32-CAM web interface  
- **quirc** – QR code detection library for ESP32-CAM

---

## 📸 System Overview
![System Diagram](https://i.postimg.cc/CdyJYH5f/image.png)

---

## 👨‍💻 Team

- Jinu Joji  
- Aadith M Mathew  
- Anaswara Jaykumar  
- Mekha Manosh  
- Aayushi Maji  

---

## 📄 License
This project is licensed under the MIT License – see the [LICENSE](./LICENSE) file for details.
