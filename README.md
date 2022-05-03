# motors_eden

Dynamixel package for Eden Robotics.

## Installation

First you need to download the Dynamixel sdk repository on you computer and init it.

```console
~ $ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
~ $ cd DynamixelSDK/python
~/DynamixelSDK/python $ python3 setup.py
```

Then clone this repository on you workspace. Make the repository and source the setup.

```console
~ $ cd eden_ws/src
~/eden_ws/src $ git clone https://github.com/AlessandriniAntoine/motors_eden.git
~/eden_ws/src $ cd ..
~/eden_ws $ catkin_make && source devel/setup.zsh
```

## Run

Open a terminal and launch ROS1.

```console
~ $ source /opt/ros/noetic/setup.zsh && roscore 
```

Open an other terminal and source the setup of you workspace and run the package

```console
~ $ cd eden_ws && source devel/setup.zsh
~/eden_ws $ rosrun motors motorsNode.py
```

## Hardware

- U2D2 dynamixel
- dynamixel XL-430 (*2)
- dynamixel XM-430
- dynamixel XC-430

## Software

- Python 3
- Ros1 noetic
- Dynamixel sdk
