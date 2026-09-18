# MN 207: Mobile Robots — Lab Assignments

Laboratory assignments, firmware, hardware documentation, and reports for **MN 207: Mobile Robots** at the **Indian Institute of Science (IISc), Bengaluru**.

**Author:** Abha Singh Sardar  
**SR Number:** 27086  
**Course:** MN 207 — Mobile Robots  

---

## Overview

| Assignment | Topic | Deliverables |
| :--- | :--- | :--- |
| **Assignment 1** | Sensor Interfacing & Characterization | IR sensor array & ultrasonic sensor reports, circuit schematics |
| **Assignment 2** | Dual H-Bridge Motor Control | L298N driver firmware (`.ino`), hardware photos, technical report |

---

## Repository Structure

```text
.
├── Assignment1/
│   ├── report_IR_Sensor.pdf           # Infrared sensor characterization report
│   ├── report_IR_Sensor.tex           # LaTeX source (IR sensor)
│   ├── IR.png                         # IR sensor circuit diagram
│   ├── report.pdf                     # Ultrasonic distance measurement report
│   ├── report_Ultrasonic_Sensor.tex   # LaTeX source (Ultrasonic sensor)
│   └── Ultra.png                      # Ultrasonic sensor circuit diagram
│
├── Assignment2/
│   ├── CODE_H_BRIDGE.ino              # Arduino firmware for L298N motor driver
│   ├── report_H_Bridge.pdf            # Motor speed & direction control report
│   ├── report_H_Bridge.tex            # LaTeX source (H-Bridge)
│   ├── hardware_front.jpeg            # Hardware setup (front view)
│   ├── hardware_side.jpeg             # Hardware setup (side view)
│   └── hardware_top.jpeg              # Hardware setup (top view)
│
├── .gitignore
└── README.md
```

---

## Assignment Summary

### Assignment 1: Sensor Characterization & Measurement
- **5-Channel Infrared Sensor Array**: Optical surface reflectance detection, analog calibration, and differential steering logic for autonomous line following.
- **Ultrasonic Sensor Module**: Time-of-flight acoustic pulse timing, distance calculation, error analysis, and obstacle detection using Arduino Mega 2560.

### Assignment 2: Dual H-Bridge Motor Driver
- **Bidirectional DC Motor Control**: L298N dual full-bridge driver interface regulating wheel velocity via PWM and directional polarity via digital logic.
- **Actuation & Hardware Testing**: Firmware implementation demonstrating forward drive, dynamic braking, reverse rotation, and differential locomotion on mobile robotic chassis.
