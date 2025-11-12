# 🤖 ESP32 Robotic Arm – Dual Joystick Controlled 

A 5-DOF robotic arm controlled by two analog joysticks using an **ESP32**, **PCA9685 servo driver**, and **high-torque DS3218MG servos**.  
Developed in **ESP-IDF (FreeRTOS)** with smooth, position-holding motion and toggle-based gripper control.

---

## 🦾 Features
- **5 servo axes**
  - Base, Shoulder, Elbow (DS3218MG)
  - Wrist, Gripper (MG90S)
- **Dual joystick control**
  - J1 X → Base  
  - J1 Y → Shoulder  
  - J2 X → Wrist  
  - J2 Y → Elbow  
  - Press **both joystick buttons once** → Close gripper  
  - Press **both again** → Open gripper  
- **Position hold** (servos retain last position)
- **Smooth PWM control** at 50 Hz using PCA9685
- **Power-efficient toggle gripper** (default open)

---

## ⚙️ Hardware Setup

| Component | Description |
|------------|-------------|
| ESP32 Dev Board | Main controller |
| PCA9685 16-bit | Servo driver (I²C @ 0x70) |
| 3× DS3218MG | Base, Shoulder, Elbow |
| 2× MG90S | Wrist, Gripper |
| 2× Joysticks | Analog X/Y + push buttons |
| 7.4 V Li-Po | Main power source |
| 8 A Buck Converter | 5–6 V for servos |
| 5 A Buck Converter | 5 V for ESP32 |
| Fuse + Switch | Power safety and control |
| Common Ground | All grounds connected together |

---

## 🧠 Control Logic

- Servos receive **PWM at 50 Hz (20 ms period)** — standard for RC servos.  
- ESP32 reads each joystick axis (0–4095 ADC) → converted to ±1 normalized range.  
- Each axis updates servo angle with smooth rate control.  
- Gripper toggles between open/closed when both joystick buttons are pressed simultaneously.

```c
// Toggle gripper on both-button press
if (both_pressed && !prev_both) {
    gripper_closed = !gripper_closed;
}
gripper_us = gripper_closed ? GRIPPER_ACTIVE_US : GRIPPER_HOME_US;
```

---

## 🧩 Pin Mapping

| Function | GPIO | Description |
|-----------|------|-------------|
| SDA | 21 | I²C Data |
| SCL | 22 | I²C Clock |
| J1 X | 34 | Base |
| J1 Y | 35 | Shoulder |
| J2 X | 32 | Wrist |
| J2 Y | 33 | Elbow |
| J1 Button | 25 | Input (pull-up) |
| J2 Button | 26 | Input (pull-up) |

| Servo | PCA9685 Channel | Port Label |
|--------|------------------|------------|
| Gripper | 4 | P5 |
| Wrist | 6 | P7 |
| Elbow | 7 | P8 |
| Base | 8 | P9 |
| Shoulder | 9 | P10 |

---

## 🧱 Mechanical Design
- Base box holds **ESP32**, **buck converters**, **fuse**, and **battery**.  
- One **15 cm arm joint** (shoulder).  
- One **13 cm arm joint** (elbow).
- **MG90S wrist + servo-driven gripper**.   
- Weighted base improves stability.

---

## 🚀 Build & Flash
```bash
idf.py build
idf.py -p COMx flash
idf.py monitor
```
> Replace `COMx` with your ESP32’s serial port.

---

## 📄 License
Released under the **MIT License**.

---

**Author:** Efe Kurnaz  
**University:** Politechnika Wrocławska  
**Project:** Engineering Thesis – Robotic Arm  
**Platform:** ESP-IDF + PCA9685 + Dual Joystick Control
