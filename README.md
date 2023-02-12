# Shaft Echange
Code for grasp two types of shaft from a box and for moving the robot toward an exchange position. 

# Requirements
The code works with a Franka Emika Panda Robot and requires the libfranka library to be installed in the system.

## Download CNN model
Download the model file from [here](https://drive.google.com/drive/folders/188HAK26zR-g8eQ3bErxWhYbbTgi29Y0Z?usp=share_link) and put it in resources/model folder.

## Change motion time
In resources/initPose.txt you can change the motion time.

# Installation
You have to change some absolute paths:

In CMakeLists.txt
Rows 34, 72

In include/utils_cam_uca.hpp
Rows 202, 237, 240, 242

In detection_aspirazione.cpp
Rows 35, 46, 48

In detection_scarico.cpp
Rows 35, 46, 48

In graspAspirazione.cpp
Rows 55, 69, 75, 163

In graspScarico.cpp
Rows 53, 67, 73, 157

In moveCobot.cpp
Rows 52, 68, 74, 169, 171

In openGripper.cpp
Rows 16, 28


To build, open a terminal and run
```shell
cd libfranka/build
make
```

To test the detector, run ONLY the code that does not move the robot:
```shell
cd libfranka/build/shaftExchange
./detection_scarico <robot_ip>
./graspScarico <robot_ip> <path_to shatf.txt file>
```

