# Micromouse Maze Solving Robot – Autonomous Navigation with IR Sensors, LiDAR, and Encoders

This project implements a **basic Micromouse robot** that autonomously navigates a maze using:
- **IR sensors** for wall detection
- **LiDAR sensor (VL53L0X)** for front obstacle detection
- **Motor encoders** for real-time speed correction

The robot follows walls, avoids obstacles, and corrects its path dynamically based on encoder feedback to maintain straight movement.

## Features
- **Real-Time Obstacle Detection:** Stops and makes smart decisions based on surrounding walls.
- **Wall Following Behavior:** Minor speed corrections to follow walls accurately.
- **Encoder-Based Speed Correction:** Balances left and right wheel speeds to maintain straight motion.
- **Basic Maze Adaptation:** Turns left, right, or defaults based on wall sensor readings.

## Technologies Used
- Arduino IDE
- ESP32
- VL53L0X LiDAR
- IR Wall Sensors
- Motor Drivers (H-Bridge)
- Hardware Interrupts for Encoders

