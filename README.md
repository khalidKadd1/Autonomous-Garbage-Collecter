# Autonomous-Garbage-Collecter
This project involves a robot designed to autonomously collect trash using a combination of computer vision, fuzzy control, and hand gesture recognition. The system is built on a Raspberry Pi and ESP32, which communicate wirelessly to control the rover's movement and the robotic arm.

Project Overview
The rover uses computer vision for object detection (specifically YOLOv8) and fuzzy control for managing distance and steering. Hand gestures are detected via computer vision to control the robotic arm. The main goal of this project is to autonomously collect bottles and cans.

Key Components:
Raspberry Pi 5: Acts as the central control unit, running object detection, fuzzy control for movement, and controlling the rover’s steering.

ESP32: Used for wireless communication and controlling the robotic arm based on hand gestures.

YOLOv8 Model: For real-time object detection of bottles and cans.

Pi camera V2: Used for object detection.

MacBook Web Cam: Hand gesture recognition to control the robotic arm.

Fuzzy Control: Applied for distance and steering control to ensure smooth navigation and accurate movements.
