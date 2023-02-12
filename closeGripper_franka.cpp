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
  if (argc != 3) {
    //std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << " <shaft -1 scarico .. 1 aspirazione> "  << std::endl;
    file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }

  std::string robot_ip = argv[1];

  int ALBERO = std::stoi(argv[2]);
  
  franka::Robot robot(robot_ip);
  franka::Gripper gripper(robot_ip);
  franka::GripperState gripper_state=gripper.readOnce();

  file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");

  float shaft_width = 0.0;
  try{
    //if (!gripper.grasp(0.0080, 0.1, 100)) { //Valore di Grasp per il gripper di Michelangelo
    //if (!gripper.grasp(0.0418, 0.1, 100)) { //Valore di Grasp per il gripper di CRF
    if (ALBERO == -1) {
        shaft_width = 0.0140921; //0.0010;
    } else if (ALBERO == 1) {
        shaft_width = 0.0140921; //0.0010;
    }
    if (!gripper.grasp(shaft_width, 0.1, 100)) { //Valore di Grasp per il gripper di Michelangelo
      if (!gripper.move(gripper_state.max_width, 0.1)) {
          fprintf(file0_, "NOK");
          fclose(file0_);
          return -1;
      }
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

