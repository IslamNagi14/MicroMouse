# Micromouse Maze Solver

A comprehensive micromouse project implementing flood-fill algorithm for maze navigation using Arduino and ESP32.

📁 Project Structure
text
micromouse/
├── Movement.c/.h           # Motor control and movement functions
├── FloodFill.c             # Maze solving algorithm implementation
├── QueMaze.h               # Maze data structures and navigation logic
├── mouse.h                 # Hardware pin definitions and configurations
├── StdTypes.h              # Custom standard type definitions
└── Mouse.c.ino            # Main Arduino sketch
🚀 Features
Flood-Fill Algorithm: Intelligent maze solving using breadth-first search

Sensor Integration: IR sensors for wall detection and navigation

Encoder-based Movement: Precise distance measurement using rotary encoders

Modular Design: Separated concerns for maze logic, movement, and hardware

Multiple Operation Modes: Dip switch controlled navigation modes

🛠️ Hardware Requirements
Microcontroller
ESP32 Development Board

Motors & Drivers
2x DC Motors with H-Bridge motor driver

PWM-controlled motor speed

Sensors
6x IR sensors (front, sides, back)

2x Rotary encoders for distance measurement

2x Dip switches for mode selection

Potentiometer for speed control

Pin Configuration (see mouse.h)
Motor Pins: 18, 19, 5, 17

IR Sensors: 25, 33, 32, 35, 34, 39

Encoders: 14, 26

Dip Switches: 16, 4

Buzzer: 27

I2C: 21 (SDA), 22 (SCL)

🔧 Installation & Setup
Clone the repository

bash
git clone https://github.com/IslamNagi14/micromouse.git
cd micromouse
Install Arduino IDE with ESP32 support

Add ESP32 board support via Board Manager

Select "ESP32 Dev Module" as target board

Install required libraries

BluetoothSerial (included with ESP32)

I2Cdev

MPU6050_6Axis_MotionApps20

Upload the code

Open Mouse.c.ino in Arduino IDE

Connect ESP32 via USB

Select correct port and upload

🎮 Operation Modes
The robot operates in three modes controlled by dip switches:

Mode 0: Autonomous Navigation
Robot starts at position (15, 0) facing North

Uses flood-fill algorithm to navigate to goal (7, 7)

Continuously updates maze map based on sensor readings

Mode 1: Pre-programmed Path
Follows predetermined path to goal

Useful for testing and calibration

Mode 2: Manual Wall Following
Implements wall-following algorithm

Makes decisions based on sensor inputs:

Right-hand rule preference

Obstacle avoidance

Dead-end handling

📊 Maze Configuration
Size: Configurable up to 20×20 cells (default 16×16)

Goal Area: 2×2 center cells (default rows 7-8, cols 7-8)

Wall Detection: Uses IR sensors to detect and map walls

Distance Calculation: Flood-fill algorithm computes shortest paths

🔄 Movement System
Motor Control
PWM-based speed control (100-200 range)

Encoder feedback for precise distance measurement

PID-like adjustment for straight-line movement

Turning Mechanisms
90° Left/Right Turns: Timed motor control

180° Turn: Combined turn operation

Orientation Tracking: Maintains current facing direction

🧠 Algorithm Details
Flood-Fill Implementation
c
// Key functions:
void Maze_FloodFill(void);           // Recalculates distances
void Maze_DetectAndSetWalls(...);    // Updates wall data
RobotDirection_t ChooseBestDirection(...); // Selects optimal move
Navigation Logic
Read current sensor values

Update maze wall data

Recalculate flood-fill distances

Choose direction with lowest distance value

Execute movement

Repeat until goal reached

📈 Performance Optimizations
Circular Queue: Efficient BFS implementation for flood-fill

Sensor Filtering: Debounced sensor readings

Adaptive Speed: Potentiometer-controlled movement speed

Encoder Balancing: Real-time motor adjustment for straight movement

🐛 Troubleshooting
Common Issues
Motors not moving: Check motor connections and PWM pins

Sensors not detecting: Verify IR sensor thresholds and wiring

Encoder errors: Ensure interrupt pins are correctly configured

Maze solving failures: Calibrate sensor thresholds and movement timing

Calibration
Adjust MAX_NUM_OF_PULSES for encoder distance calibration

Modify turn delay values in movement functions

Tune IR sensor thresholds in mouse.h

🔮 Future Enhancements
MPU6050 integration for gyro-assisted turning

Bluetooth remote control and monitoring

Optimized path smoothing algorithms

Race mode for speed optimization

LCD display for real-time status

👥 Contributors
Islam Nagi - Project lead and main developer

📄 License
This project is open source. Feel free to modify and distribute for educational and competition purposes.

Note: This code is designed for micromouse competitions and may require calibration for specific hardware configurations and maze environments.

