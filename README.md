# Maze-Solving Robot

A maze-solving robot built with ESP32 that uses the **Left-Hand Rule algorithm** to navigate through physical mazes.

## Overview

This robot was built and tested in a real maze environment. It uses distance and infrared sensors to detect walls and navigate autonomously from start to goal.

## Hardware Components

| Component | Purpose |
|-----------|---------|
| ESP32 | Main microcontroller |
| VL53L0X | Front distance sensor (wall detection) |
| IR Sensors (x2) | Left and right wall detection |
| MPU6050 | Gyroscope/Accelerometer |
| DC Motors (x2) | Movement |
| Motor Encoders | Distance tracking |
| L298N Motor Driver | Motor control |

## Pin Configuration

```
Right Motor: ENA=14, IN1=32, IN2=33
Left Motor:  ENB=12, IN3=26, IN4=18
Encoders:    Left=25, Right=16
IR Sensors:  Left=5, Right=4
```

## How It Works

1. **Left-Hand Rule**: The robot follows the left wall to navigate the maze
2. **Wall Detection**: VL53L0X detects front walls, IR sensors detect side walls
3. **Movement**: Encoders ensure precise cell-by-cell movement (20cm per cell)
4. **Goal Detection**: Robot stops when it reaches the goal position

## Maze Configuration

- Grid size: 8x8 cells
- Cell size: 20cm x 20cm
- Wall detection threshold: 80mm

## Algorithm Priority

When deciding the next move, the robot checks directions in this order:
1. Left
2. Straight
3. Right
4. Back (180° turn)

## Libraries Required

- Wire.h
- Adafruit_Sensor
- VL53L0X
- I2Cdev
- MPU6050

## Usage

1. Upload `mazeRobot.ino` to your ESP32
2. Place the robot at the maze entrance
3. Power on - the robot will start solving automatically
4. Robot stops when goal is reached


