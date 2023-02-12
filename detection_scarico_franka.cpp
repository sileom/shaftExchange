// *************** LIBRERIE
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include "include/common_uca_2.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <math.h>

#include "include/dynModel_uca_2.h"
#include "include/utils_uca_2.hpp"
#include "include/utils_cam_uca_2.hpp"
  
#include <ctime>
#include <stdio.h>
#include <string>

// *************** DICHIARAZIONE FUNZIONI
int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf);

// *************** VARIABILI PER IL SALVATAGGIO SU FILE
FILE* file0_;

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  if (argc != 2) {
    //std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }

  std::string robot_ip = argv[1];
  int ALBERO = -1; //stoi(argv[2]);
  franka::Robot robot(robot_ip);
  franka::RobotState robot_state = robot.readOnce();
  
  file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");

  std::string nome= "/home/labarea-franka/libfranka/uca_franka_lib1/resources/initPose.txt";
  std::vector<Eigen::Matrix3d> Rf_vec;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(3,3);
  int numPti=-1;
  double tempo = -1.0;
  Rf_vec = readPointsFile(robot_state, nome, &numPti, &tempo, &P);

  //Eigen::Matrix3d R_PROVA = Eigen::Matrix3d::Identity();


  move(robot_ip, Eigen::Vector3d(P(1,0),P(1,1),P(1,2)), Rf_vec[1], 0, tempo);
//LANCIO IL DETECTOR
  int esito_detection = -1;
  esito_detection = mainMethod(ALBERO);
  if (esito_detection == 0){
    fprintf(file0_, "OK");
    fclose(file0_);
  } else {
    fprintf(file0_, "NOK");
    fclose(file0_);
  }
  
  return 0;
}



int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf){
  Eigen::MatrixXd K;
  K.setIdentity(6,6);
  K(0,0) = 100;     K(1,1) = 100;     K(2,2) = 100;     K(3,3) = 10;      K(4,4) = 10;     K(5,5) = 10;
   
  try {
    franka::Robot robot(robot_ip);
    franka::Model model = robot.loadModel();
    franka::RobotState robot_state = robot.readOnce();
    setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose = robot_state.O_T_EE;
    std::array<double, 16> posa_desiderata = initial_pose;

    //DIFINIZIONE SPOSTAMENTO CARTESIANO DESIDERATO
    posa_desiderata[12] = final_point[0];     //X
    posa_desiderata[13] = final_point[1];     //Y
    posa_desiderata[14] = final_point[2];     //Z

    //calcolo della traiettoria polinomiale
    std::array<double, 4> coeff_pos_x{};
    std::array<double, 4> coeff_pos_y{};
    std::array<double, 4> coeff_pos_z{};
    std::array<double, 3> coeff_vel_x{};
    std::array<double, 3> coeff_vel_y{};
    std::array<double, 3> coeff_vel_z{};
    calcolaCoeffTraiettoriaPos(initial_pose[12], posa_desiderata[12], 0.0, 0.0, ti, tf, coeff_pos_x);
    calcolaCoeffTraiettoriaPos(initial_pose[13], posa_desiderata[13], 0.0, 0.0, ti, tf, coeff_pos_y);
    calcolaCoeffTraiettoriaPos(initial_pose[14], posa_desiderata[14], 0.0, 0.0, ti, tf, coeff_pos_z);
    calcolaCoeffTraiettoriaVel(initial_pose[12], posa_desiderata[12], 0.0, 0.0, ti, tf, coeff_vel_x);
    calcolaCoeffTraiettoriaVel(initial_pose[13], posa_desiderata[13], 0.0, 0.0, ti, tf, coeff_vel_y);
    calcolaCoeffTraiettoriaVel(initial_pose[14], posa_desiderata[14], 0.0, 0.0, ti, tf, coeff_vel_z);


    //PASSIAMO IN RAPPRESENTAZIONE ASSE-ANGOLO E PIANIFICHIAMO L'ORIENTAMENTO
    Eigen::MatrixXd R0 = getR(initial_pose);
    Eigen::MatrixXd Rf = final_matrix; 
    //STAMPA PER VEDERE SE ASSEGNA BENE LE MATRICI
    //cout << Rf << endl;

    Eigen::MatrixXd Rf0 = R0.transpose()*Rf;
    Eigen::VectorXd r_theta = r2asseangolo(Rf0);
    Eigen::Vector3d r(r_theta[0], r_theta[1], r_theta[2]);
    double theta_f = r_theta[3]; 

    Eigen::Vector3d p_des;
    Eigen::Vector3d v_des;
    Eigen::VectorXd s_ds;
    Eigen::MatrixXd R_d;
    Eigen::Vector3d omegad;

    double time = 0.0;

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities { 
      time += period.toSec();

      if(time < (tf-ti) ) {
        p_des = calcolaPolPos(ti, time, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolVel(ti, time, coeff_vel_x, coeff_vel_y, coeff_vel_z);
        s_ds = ascissa(time, tf);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
      } else {
        //std::cout << "Assestamento" << std::endl;
        p_des = calcolaPolPos(ti, tf, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolVel(ti, tf, coeff_vel_x, coeff_vel_y, coeff_vel_z);
        s_ds = ascissa(tf, tf);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
      }

      franka::CartesianVelocities output = {{v_des[0], v_des[1], v_des[2], omegad[0], omegad[1], omegad[2]}};
      //franka::CartesianVelocities output = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };

      if (time >= ((tf-ti)+t_ass)) {
        std::cout << std::endl << "Finished motion, shutting down robot"  << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}




 
