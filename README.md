# Autonomous Mobile Robot with Computer Vision

An autonomous mobile robot project that combines **ROS**, **Arduino-based motor control**, and **OpenCV** for sensor-driven navigation and interactive human-robot behavior.
<img width="404" height="343" alt="image" src="https://github.com/user-attachments/assets/93c93689-7b34-4f53-9a50-73bc2aaf39ed" />

---

## Overview

This project was developed as an integrated robotics system with two main capabilities:

- **Autonomous navigation using light and IR beacon signals**
- **Interactive photo capture using hand, face, and smile detection**

The system uses a **two-layer architecture**:

- **Arduino** handles low-level motor control, encoder feedback, PID speed control, and sensor publishing.
- **ROS** handles high-level navigation logic, behavior switching, computer vision, and photo capture.

---

## Key Features

- Differential-drive mobile robot control
- PID-based wheel speed regulation
- Encoder-based closed-loop motion control
- Light sensor and IR receiver integration
- Bumper-based obstacle response
- ROS state-machine navigation
- Hand-triggered interaction mode
- Face and smile detection using OpenCV
- Automatic photo capture and local image saving

---

## System Architecture

```text
+-------------------------------+
|        High-Level Layer       |
|        ROS + OpenCV           |
|-------------------------------|
| light_receive_data.cpp        |
| - Navigation state machine    |
| - Beacon search / approach    |
|                               |
| smile_photo_node.cpp          |
| - Hand detection              |
| - Face / smile detection      |
| - Auto photo capture          |
+---------------+---------------+
                |
                | ROS topics / serial communication
                v
+-------------------------------+
|        Low-Level Layer        |
|           Arduino            |
|-------------------------------|
| light_receive_data.ino        |
| - Light / IR sensing          |
| - PID motor control           |
|                               |
| smile_photo_node.ino          |
| - cmd_vel motor execution     |
| - Encoder-based PID control   |
+---------------+---------------+
                |
                v
+-------------------------------+
|            Hardware           |
|-------------------------------|
| L298N motor driver            |
| DC motors + encoders          |
| Light sensor                  |
| IR receiver                   |
| Bumper switches               |
| USB camera                    |
+-------------------------------+
```

---

## Project Structure

```text
.
├── light_receive_data.cpp      # ROS node for light / IR beacon navigation
├── smile_photo_node.cpp        # ROS node for hand / face / smile interaction
├── light_receive_data.ino      # Arduino firmware for navigation mode
└── smile_photo_node.ino        # Arduino firmware for motion control in photo mode
```

---

## File Descriptions

### `light_receive_data.cpp`
ROS navigation node for autonomous movement.

Main responsibilities:
- subscribes to `light_data` and `ir_ratio`
- publishes `cmd_vel`
- reads bumper and GPIO input
- controls robot behavior using a state machine
- handles beacon search, approach, fallback, and obstacle recovery

### `light_receive_data.ino`
Arduino firmware for light / IR navigation mode.

Main responsibilities:
- reads analog light sensor data
- measures IR low-ratio signal
- publishes sensor data to ROS
- receives `/cmd_vel`
- controls wheel motors with PID and encoder feedback

### `smile_photo_node.cpp`
ROS vision node for interactive photo capture.

Main responsibilities:
- captures live video from camera
- detects hand gesture as an interaction trigger
- detects face and smile using OpenCV Haar cascades
- adjusts robot position for better framing
- saves captured photos automatically

### `smile_photo_node.ino`
Arduino firmware for photo interaction mode.

Main responsibilities:
- receives `/cmd_vel` from ROS
- controls differential-drive motion
- uses encoder-based PID speed control
- stops safely if command updates are lost

---

## Hardware

- Raspberry Pi or Linux host running ROS
- Arduino Uno
- L298N motor driver
- DC motors with encoders
- Differential-drive chassis
- Analog light sensor
- IR receiver module
- Bumper switches
- USB camera

---

## Software Stack

- ROS 1
- C++
- Arduino
- OpenCV
- rosserial
- PID_v1 library
- wiringPi

---

## Main Modes

### 1. Light / IR Beacon Navigation
In this mode, the robot moves autonomously while monitoring:

- light intensity
- IR beacon ratio
- bumper input

The robot uses a state machine to:
- wait for start
- move forward
- recover from collisions
- search for a beacon
- approach the beacon when detected

### 2. Smile Photo Interaction
In this mode, the robot behaves like an interactive photo robot.

Workflow:
1. patrols by turning and pausing
2. waits for a hand trigger
3. switches to face and smile detection
4. adjusts position if framing is poor
5. captures a photo when a smile is detected consistently
6. enters cooldown before restarting

---

## ROS Topics

### Published
- `light_data` (`std_msgs/Int16`)
- `ir_ratio` (`std_msgs/Float32`)
- `cmd_vel` (`geometry_msgs/Twist`)

### Used by the system
- sensor data from Arduino to ROS
- motion commands from ROS to Arduino

---

## Dependencies

Before running the project, make sure the following are installed:

- ROS 1 environment
- OpenCV with Haar cascade files
- rosserial
- wiringPi
- Arduino IDE or Arduino CLI
- PID_v1 Arduino library

Typical cascade files used in this project:
- `haarcascade_frontalface_default.xml`
- `haarcascade_smile.xml`

---

## Build and Run

### 1. Upload Arduino firmware
Choose the correct firmware depending on the demo mode:

- `light_receive_data.ino` for navigation mode
- `smile_photo_node.ino` for smile-photo mode

### 2. Build your ROS workspace

```bash
catkin_make
source devel/setup.bash
```

### 3. Run the ROS node

Navigation mode:

```bash
rosrun your_package light_receive_data
```

Smile photo mode:

```bash
rosrun your_package smile_photo_node
```

> Replace `your_package` with your actual ROS package name.

---

## Example Applications

- Autonomous mobile robot demo
- Interactive exhibition robot
- Robotics course project
- Embedded systems + ROS integration project
- Computer vision based human-robot interaction prototype

---

## Future Improvements

- Replace Haar cascades with deep learning-based detection
- Improve navigation robustness in complex environments
- Add SLAM or localization support
- Add remote monitoring or dashboard interface
- Support voice interaction
- Store metadata together with captured photos

---

## Notes

This repository demonstrates the integration of:

- embedded motor control
- sensor feedback
- state-machine-based autonomy
- ROS communication
- vision-based interaction

It is suitable for academic presentation, portfolio use, and further robotics development.

---

## License

This project is shared for academic and portfolio demonstration purposes.
