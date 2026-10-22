# MN 207: Mobile Robots — Lab Assignments

Laboratory assignments, firmware, hardware documentation, and reports for **MN 207: Mobile Robots** at the **Indian Institute of Science (IISc), Bengaluru**.

---

## Overview

| Assignment | Topic | Deliverables |
| :--- | :--- | :--- |
| **Assignment 1** | Sensor Interfacing & Characterization | IR sensor array & ultrasonic sensor reports, circuit schematics |
| **Assignment 2** | Dual H-Bridge Motor Control | L298N driver firmware (`.ino`), hardware photos, technical report |
| **Assignment 3** | Forward Kinematics & Wheel Odometry | Differential drive odometry firmware, hardware test photos, serial telemetry report |
| **Assignment 4** | Dual Motor PID Speed Control | Discrete dual PID control firmware (`.ino`), optical encoder feedback, 6-page technical report |

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
├── Assignment3/
│   ├── code.ino                       # Arduino sketch for kinematics & odometry
│   ├── report_Kinematics_Odometry.pdf # Technical report (8-page validated PDF)
│   ├── report_Kinematics_Odometry.tex # LaTeX source for Assignment 3 report
│   ├── lab4.png                       # Bench test hardware and telemetry setup
│   ├── lab4_1.png                     # Autonomous mobile robot ground navigation
│   ├── Lab4_Result.png                # Serial monitor telemetry capture
│   └── example assignment.pdf         # Reference assignment documentation
│
├── Assignment4/
│   ├── PID_CONTROL.ino                # Arduino firmware for dual closed-loop PID control
│   ├── report_PID_Control.pdf         # Technical report (6-page validated PDF)
│   └── report_PID_Control.tex         # LaTeX source for Assignment 4 report
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

### Assignment 3: Forward Kinematics & Wheel Odometry
- **Differential Drive Kinematics**: Optical slot encoder dead reckoning estimating pose $(x, y, \theta)$ and body/world velocities $(v, \omega, v_x, v_y)$.
- **Firmware Engineering Optimizations**: Software debouncing ($1500\,\mu\text{s}$), direction-aware count accumulation, update gating ($30\,\text{ms}$), midpoint heading integration, and fail-safe timeouts.
- **Trajectory Validation**: Autonomous benchmark execution ($1.0\,\text{m}$ forward $\to 90^\circ$ right turn $\to 0.5\,\text{m}$ forward) achieving $1.20\,\text{cm}$ translational accuracy ($0.80\%$ error) and $1.35^\circ$ heading accuracy.

### Assignment 4: Dual Motor Closed-Loop PID Speed Control
- **Optical Encoder Feedback**: Real-time pulse interrupt processing with $1000\,\mu\text{s}$ refractory debouncing, CPR calibration ($\text{CPR}_L = 41.95, \text{CPR}_R = 40.27$), and first-order EMA low-pass velocity smoothing ($\alpha = 0.15$).
- **Discrete Dual PID with Anti-Windup**: Independent concurrent digital PID controllers with conditional saturation gating ($0 < u[k-1] < 255$) and strict integral accumulator clamping ($[-150, +150]$) to eliminate startup overshoot.
- **Actuator Asymmetry Compensation**: Autonomous dynamic PWM balancing compensating for physical motor discrepancy ($450\,\text{RPM}$ vs $390\,\text{RPM}$), regulating both wheels to an identical steady-state target velocity of $200.0\,\text{RPM}$ with zero steady-state error.

