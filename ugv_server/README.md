# Description
This is a personalisation use of the example given by the [waveshareteam/ugv_jetson](https://github.com/waveshareteam/ugv_jetson) repository.

For more information see the [waveshareteam/ugv_jetson](https://github.com/waveshareteam/ugv_jetson) repository.

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the upper computer, which is in our case a Jetson Orin NX in this setup.  

The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used.  

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a Jetson Orin NX, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

## Features
- Real-time video based on WebRTC
- Pan-tilt camera control
- Robotic arm control
- Cross-platform web application base on Flask
- Auto targeting (OpenCV)
- Object Recognition (OpenCV)
- Gesture Recognition (MediaPipe)
- Face detection (OpenCV & MediaPipe)
- Motion detection (OpenCV)
- Line tracking base on vision (OpenCV)
- Color Recognition (OpenCV)
- Multi-threaded CV processing
- Audio interactive
- Shortcut key control
- Photo taking
- Video Recording
- Real-time position tracking

## Quick Install
See [ugv_jetson's Quick Install](https://github.com/waveshareteam/ugv_jetson#quick-install) repository.

You can access the robot web app using a mobile phone or PC. Simply open your browser and enter `[IP]:5000` (for example, `192.168.10.50:5000`) in the URL bar to control the robot.  

If the robot is not connected to a known WiFi network, it will automatically set up a hotspot named "`AccessPopup`" with the password `1234567890`. You can then use a mobile phone or PC to connect to this hotspot. Once connected, open your browser and enter `192.168.50.5:5000` in the URL bar to control the robot.  

# License
ugv_server Jetson Orin NX: an open source robotics platform for the Jetson Developer Kit derived from the ugv_jetson project.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
