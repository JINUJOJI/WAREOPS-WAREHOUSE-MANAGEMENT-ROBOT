# WAREOPS-WAREHOUSE-MANAGEMENT-ROBOT
# WAREOPS: A Warehouse Management Robot

WAREOPS is a line-following, pick-and-place warehouse management robot developed as part of an engineering project at VIT-AP University. The system is designed to automate item transport within a warehouse using sensors, servo-controlled lifting mechanisms, and wireless communication.

---

## 💡 Project Objective

To optimize warehouse operations by automating the movement and placement of packages using a robot equipped with:

- Infrared sensors for navigation
- Ultrasonic sensors for obstacle detection
- QR code-based route decision system
- Wi-Fi modules for communication with smart racks and user interface

---

## 🔧 Features

- **Line Following Navigation:**  
  The robot uses IR sensors to follow a predefined path within the warehouse.

- **Pick and Place Mechanism:**  
  A servo-controlled forklift system lifts and places boxes based on predefined height values (~4.5 cm at 90° rotation).

- **Obstacle Detection:**  
  Dual ultrasonic sensors detect obstacles and help avoid collisions during transit.

- **QR Code-Based Routing:**  
  At intersections, the robot scans QR codes to determine the correct direction.

- **Camera-Based Object Scanning:**  
  An ESP32-CAM module captures images of objects, which are analyzed via a YOLO model to determine object dimensions.

- **Web-Based Control System:**  
  A web server is used to start/stop the robot and receive image analysis results from the YOLO model.

---

## 🛠️ Hardware Used

- PIC16F877A Microcontroller  
- ESP32-CAM Wi-Fi Module  
- ESP8266 Wi-Fi Module  
- IR Sensors  
- Ultrasonic Sensors  
- Servo Motor  
- Breadboards, jumper wires, battery pack  
- QR Code scanner (camera-based)

---

## 💻 Software Stack

- **Arduino IDE** – Programming the microcontroller  
- **HTML + Web Server** – To control robot and display scanning results  
- **Python (YOLO processing)** – On server side  
- **Embedded C** – For PIC microcontroller logic

---

## 📂 File Structure

- `/firmware` – Code for IR sensor, servo, ultrasonic
- `/esp32-cam` – Image capture and upload code
- `/esp8266` – Communication with smart rack
- `/web-server` – HTML files and scripts for control interface
- `/QR-processing` – Code for QR code recognition and routing


## 👨‍💻 Team

- Jinu Joji  
- Aadith M Mathew  
- Anaswara Jaykumar  
- Mekha Manosh  
- Aayushi Maji  
- Anurag De  

---

## 📜 License

This project is under the MIT License. See [LICENSE](./LICENSE) for details.

