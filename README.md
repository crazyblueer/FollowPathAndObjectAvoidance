This repository contains the development and simulation of a multi-tasking robot capable of performing target recognition, path following, object avoidance, and localization.
The robot utilizes a combination of Raspberry Pi, Arduino, sensors and various algorithms for real-time decision-making and task execution.

Project Overview

In this project, my teammates and I developed a robot that integrates multiple functionalities, including:
- Localization: The robot can determine its position within a given environment using lidar.
- Target Recognition: It identifies specific targets using camera (The green bottle).
- Object Picking: The robot can autonomously pick up and drop the object using servo sensor and ultrasonic sensor. 
- Path Following: The robot follows a pre-defined path using sensor data and control algorithms based on lidar scanning and ICP algorithm. 
- Obstacle Avoidance: The robot detects and avoids obstacles in real-time using ultrasonic sensor.
  
This project was implemented using a Raspberry Pi for processing, Arduino for hardware control, and a combination of C/C++ and Python for system programming.

Algorithms Implemented

1. PID Control
The robot uses a Proportional-Integral-Derivative (PID) control algorithm to follow a desired path. The algorithm adjusts motor speeds to minimize the error in position and orientation.

Implementation: Simulated and implemented on MATLAB and the ROS platform.

2. Feedback Control Laws
Feedback control algorithms were used for real-time decision-making, allowing the robot to respond to changes in the environment, such as obstacles or deviations from the desired path.

Implementation: Integrated into the control loop for obstacle avoidance and path following.

3. Iterative Closest Point (ICP)
The ICP algorithm was used for localization and mapping, enabling the robot to align sensor data with a map of the environment.

Implementation: This algorithm was implemented on MATLAB and ROS for accurate localization.
Hardware Used
Raspberry Pi: The main processing unit responsible for running the algorithms and managing communication.
Arduino: Used to interface with sensors and control motors, allowing for real-time hardware manipulation.
Sensors:
Ultrasonic and infrared sensors for obstacle detection.
Cameras for target recognition.
IMU (Inertial Measurement Unit) for localization and orientation tracking.
Software Used
MATLAB: For algorithm simulation and initial prototyping.
ROS (Robot Operating System): For robot control, communication, and simulation.
C/C++: Used for low-level programming of the robot’s hardware (Arduino and Raspberry Pi).
Python: Used for high-level control and algorithm integration.

