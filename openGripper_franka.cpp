// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "include/common_uca_2.h"

FILE* file0_;

using namespace std;

int main(int argc, char** argv) {
  if (argc != 2) {
    //std::cerr << "Usage: " << argv[0] << " <robot-hostname> "  << std::endl;
    file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }

  std::string robot_ip = argv[1];
  
  franka::Robot robot(robot_ip);
  franka::Gripper gripper(robot_ip);
  franka::GripperState gripper_state=gripper.readOnce();

  file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");

  try{
    if (!gripper.move(gripper_state.max_width, 0.1)) {
      //std::cout << "Failed open." << std::endl;
      fprintf(file0_, "NOK");
      fclose(file0_);
      return -1;
    }
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }
  fprintf(file0_, "OK");
  fclose(file0_);
  return 0;
}

