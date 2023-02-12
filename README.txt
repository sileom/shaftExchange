Download the model file from here and put it in 


In resources/initPose.txt you can change the motion time.


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


To build, open a terminal in folder libfranka/build and run
make


To test the detector, run ONLY the code that does not move the robot.
In folder build/uca_franka run first

./detection_scarico <robot_ip>

and then
./graspScarico <robot_ip> <path_to shatf.txt file>
