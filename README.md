# Mobile Robots — Lab Assignments

This repository contains lab assignments and reports for the **Mobile Robots** course (Semester 3) at the Indian Institute of Science (IISc).

**Author**: Abha Singh Sardar (SR No: 27086)

---

## Repository Structure

```
.
├── Assignment1/
│   ├── IR.png                         # IR sensor circuit / setup diagram
│   ├── Ultra.png                      # Ultrasonic sensor circuit / setup diagram
│   ├── report_IR_Sensor.tex           # LaTeX source for IR sensor report
│   ├── report_IR_Sensor.pdf           # Compiled report on IR sensor characterization
│   ├── report_Ultrasonic_Sensor.tex   # LaTeX source for Ultrasonic sensor report
│   └── report.pdf                     # Compiled report on Ultrasonic distance measurement
│
├── Assignment2/
│   ├── CODE_H_BRIDGE.ino              # Arduino firmware for H-Bridge motor control
│   ├── hardware_front.jpeg            # Hardware setup (front view)
│   ├── hardware_side.jpeg             # Hardware setup (side view)
│   ├── hardware_top.jpeg              # Hardware setup (top view)
│   ├── report_H_Bridge.tex            # LaTeX source for H-Bridge driver report
│   └── report_H_Bridge.pdf            # Compiled report on H-Bridge motor driver
│
├── .gitignore
└── README.md
```

---

## Assignment Summaries

### Assignment 1: Sensor Characterization & Measurement
- **IR Sensor Module**: Characterization of infrared distance sensing, calibration curves, and ADC value mapping.
- **Ultrasonic Sensor Module**: Distance measurement utilizing time-of-flight acoustic pulses, pulse timing via Arduino microcontroller, error analysis, and obstacle detection.

### Assignment 2: H-Bridge Motor Driver
- **Design & Operation**: Bi-directional DC motor control using an H-Bridge configuration.
- **Firmware Implementation**: Arduino C++ implementation (`CODE_H_BRIDGE.ino`) demonstrating forward, reverse, and braking states with PWM speed modulation.
- **Hardware Validation**: Complete breadboard/chassis circuit assembly and demonstration report.
