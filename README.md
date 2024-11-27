# Line-Following Robot with PD Control and Obstacle Detection

A line-following robot designed to track a line using five analog sensors, avoid obstacles using a sonar sensor, and steer with precise control using PD (Proportional-Derivative) algorithms. The robot uses two motors controlled by an H-bridge driver and an Arduino microcontroller for decision-making.

---

## Features

- **Line Tracking:** Uses 5 analog sensors to detect the position of a line and calculate the error for PD control.
- **PD Control Algorithm:** Provides smooth and precise steering by adjusting motor speeds based on proportional and derivative terms.
- **Obstacle Detection:** Employs an ultrasonic sonar sensor to detect obstacles and stop or avoid them.
- **Motor Control:** H-bridge driver enables direction and speed control of two DC motors using PWM signals.

---

## Components Used

1. **Arduino Microcontroller**
2. **5 Analog Line Sensors** (connected to A0–A4)
3. **Ultrasonic Sensor** (HC-SR04 or equivalent)
4. **H-Bridge Motor Driver** (e.g., L298N or L293D)
5. **Two DC Motors**
6. **Chassis with Wheels**
7. **Power Supply** (Battery pack)

---

## Circuit Diagram

1. **Sensors:**
   - Connect 5 analog line sensors to pins A0 to A4.
2. **Sonar Sensor:**
   - Trigger pin to digital pin 6.
   - Echo pin to digital pin 7.
3. **H-Bridge:**
   - Left motor: Direction pin to digital pin 8, PWM pin to digital pin 9.
   - Right motor: Direction pin to digital pin 11, PWM pin to digital pin 10.

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/line-following-robot.git
   cd line-following-robot
2. Open the `line_following_robot.ino` file in the Arduino IDE.

3. Connect your Arduino board to your computer using a USB cable.

4. Select the correct **board** and **port** in the Arduino IDE.

5. Upload the code to your Arduino board by clicking the **Upload** button.

---

## Usage

1. Place the robot on a track with a contrasting black line on a white surface.
2. Ensure the sensors are correctly aligned to detect the line.
3. Power on the robot.
4. The robot will follow the line while avoiding obstacles.
5. If an obstacle is detected, the robot will stop or perform a pre-programmed avoidance maneuver.

---

