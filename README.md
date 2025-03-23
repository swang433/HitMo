Boxing Robot Vision Evaluation System
=====================================

Overview:
---------
This project implements a boxing robot arm that responds in real time to a user's boxing moves using computer vision. It combines Python-based image processing and a C++ Arduino-based finite state machine to control the robot's actions.

Files:
------
- main.py: Captures webcam input, performs motion detection, and sends commands via serial to the robot arm.
- NewCV_Swap.py: Alternate computer vision script using OpenCV for punch detection logic.
- FSM_full.ino: Arduino sketch implementing a finite state machine to control servo motors based on serial input.

Features:
---------
- Real-time computer vision-based punch detection
- Serial communication between Python and Arduino
- Finite state machine logic for responsive robot motion
- Modular design with separate logic for vision and control

Requirements:
-------------
- Python 3.x
- OpenCV (`pip install opencv-python`)
- PySerial (`pip install pyserial`)
- Arduino IDE
- Arduino Uno (or compatible board)
- Servo motors and boxing robot hardware setup

Usage:
------
1. Load FSM_full.ino to your Arduino via the Arduino IDE.
2. Run main.py or NewCV_Swap.py on your PC with a connected webcam.
3. The robot arm should respond in real time to detected punches.

Author:
-------
Joseph Spracklen, Esther Matarranz MArtin, Ray Wang, Fengrui Yang
