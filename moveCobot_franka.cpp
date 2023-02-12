// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include "include/common_uca_2.h"
#include "include/dynModel_uca_2.h"
#include "include/utils_uca_2.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <math.h>

#include <ctime>
#include <stdio.h>
#include <string>

bool force_lim(int seg, Eigen::Vector3d forze);
bool joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT);
 
// **************** VARIABILI GLOBALI
//tempo di inizio
const double t_i = 0.0;
//const double t_ass = 0.1;//1.5;

double ForzaLim=0; // FORZA DI CONTATTO LIMITE PER LA DETECTION DELL'ALBERO DA PRENDERE
double tau_fl=0;   // COSTANTE DI TEMPO DEL GRADINO ESPONENZIALE UTILIZZATO PER AZZERARE LE q_dot
const int SEG_RIL=6; // SEGMENTO NEL QUALE AVVIENE LO SCAMBIO
bool IS_SHAFT_ROTATE_local;
Eigen::Vector3d ffr1;


//variabili per il salvataggio su file
FILE* file0_;

int ALBERO;
bool grasp =false;

using namespace std;
using namespace Eigen;

int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf);
Eigen::MatrixXd readMeasuredPose(int albero);
int move_ee(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf);

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << "<input file path of shaft pose> " << " <num albero: -1 --> scarico(corto) .. 1 --> aspirazione> " << "Posizione: 5 -> interscambio .. 7 -> disimpegno" << std::endl;
    file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }
  
  t_ass = 0.1;

  //MoveRobotp2p move_robot;
  std::string robot_ip = argv[1];
  std::string filepath = argv[2];

  ALBERO = stoi(argv[3]);
  int posizione_op = stoi(argv[4]);
  //ROTAZIONE ALBERI
  FILE* file_rot;
  file_rot = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/isRotate.txt", "r");
  char stringa1[80];
  fscanf(file_rot,"%s",stringa1);
  int val = std::stoi(stringa1);
  IS_SHAFT_ROTATE_local = (val == 0) ? false : true;

  file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");

  franka::Robot robot(robot_ip);
  franka::Gripper gripper(robot_ip);
  franka::GripperState gripper_state=gripper.readOnce();
  franka::RobotState robot_state = robot.readOnce();

  //LETTURA PUNTO SHAFT
  FILE* file2;
  file2 = fopen(filepath.c_str(), "r");
  Eigen::MatrixXd P;
  char stringa[80];
  int res;
  int i = 1;
  int j = 0;
  res = fscanf(file2,"%s",stringa);
  int numPti = stoi(stringa) +1;
  res = fscanf(file2,"%s",stringa);
  double tempo = stod(stringa);
  std::vector<Eigen::Matrix3d> Rf_vec(numPti); 
  P.resize(numPti, 3);
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q_d.data());
  std::vector<double> Theta;
  Theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
  Eigen::MatrixXd F;
  F = fk(a, d, alpha, Theta);
  P(0,0) = F(0,3); 
  P(0,1) = F(1,3); 
  P(0,2) = F(2,3);
  P(0,3) = 0; P(0,4) = 0; P(0,4) = 0;
  Rf_vec[0] = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rd;

  res = fscanf(file2,"%s",stringa);
  while(i < numPti && res != EOF) {
    //cout << stringa << endl;
    if(strcmp(stringa, "Rd") == 0){
      Rd = readMatrix(file2);
      Rf_vec[i] = Rd;
      j = 0;
      i++;
    } else {
      P(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file2,"%s",stringa);
  }

  double velocita_perc = 0.15;

  try{
    ffr1 << 0,0,0; 

    //int seg=1;
    int esito = 2;

    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

	
    
    if (ALBERO == -1) { //scarico - corto
      if(posizione_op==5){ //interscambio

        // prova2_lab
        std::array<double, 7> q_goal = {{-0.0475716, -0.765676, 0.0694383, -2.40341, 0.0427194, 1.60732, 0.780437}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        q_goal = {{1.49755, -0.765676, 0.0694383, -2.40341, 0.0427194, 1.60732,0.780437}};
        MotionGenerator mg2(velocita_perc, q_goal);
        robot.control(mg2);

        q_goal = {{1.02388, -1.56557, 1.46486, -2.31365, 0.165188, 2.79843, 2.09603}};
        MotionGenerator mg5(velocita_perc, q_goal);
        robot.control(mg5);

      } else if(posizione_op==7){ //disimpegno
        // prova2_lab
        std::array<double, 7> q_goal = {{1.2731, -1.6130, 1.4458, -2.3502, 0.1519, 2.5801, 2.1073}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        //q_goal = {{1.49755, -0.805204, 0.279943, -2.34503, 0.196524, 1.53299, 0.908598}};
        //MotionGenerator mg2(velocita_perc, q_goal);
        //robot.control(mg2);

        //q_goal = {{-0.0475716, -0.765676, 0.0694383, -2.40341, 0.0427194, 1.60732, 0.780437}};
        //MotionGenerator mg5(velocita_perc, q_goal);
        //robot.control(mg5);

      } else {
        // Sono in condizione di errore: carico il file con la posizione nel kit
        // pos(?)
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        Eigen::MatrixXd T_kit = readMeasuredPose(ALBERO);
        Eigen::Matrix3d R_kit = T_kit.block<3,3>(0,0);
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)+0.10),R_kit, 0, tempo); 
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)),R_kit, 0, tempo); 
        if (!gripper.move(gripper_state.max_width, 0.1)) {
          //std::cout << "Failed open." << std::endl;
          esito = -1;
        }
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)+0.10),R_kit, 0, tempo); 
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
      }
    } else if (ALBERO == 1) { //aspirazione - lungo
      if(posizione_op==5){ //interscambio
        // prova2_lab
        std::array<double, 7> q_goal = {{-0.0475716, -0.765676, 0.0694383, -2.40341, 0.0427194, 1.60732, 0.780437}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        q_goal = {{-0.0473167, -1.05414, -0.891983, -2.14967, -0.743354, 1.35458, 0.893817}};
        MotionGenerator mg2(velocita_perc, q_goal);
        robot.control(mg2);

        q_goal = {{0.05, -1.05389, -1.65628, -1.28647, -1.08039, 1.4689, 0.144815}};
        MotionGenerator mg3(velocita_perc, q_goal);
        robot.control(mg3);

        q_goal = {{0.13835, -1.61645, -1.65628, -1.28647, -0.431539, 3.41286, -0.364761}};
        MotionGenerator mg7(velocita_perc, q_goal);
        robot.control(mg7);
  
      } else if(posizione_op==7){ //disimpegno
        // prova2_lab
        std::array<double, 7> q_goal = {{-0.1935, -1.5121, -1.7458, -1.7924, -0.4076, 3.5976, -0.3240}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        //q_goal = {{-0.764536, -1.05414, -0.891983, -2.14967, -0.743354, 1.35458, 0.893817}};
        //MotionGenerator mg2(velocita_perc, q_goal);
        //robot.control(mg2);

        //q_goal = {{-0.0475716, -0.765676, 0.0694383, -2.40341, 0.0427194, 1.60732, 0.780437}};
        //MotionGenerator mg5(velocita_perc, q_goal);
        //robot.control(mg5);
  
  
      } else {
        // Sono in condizione di errore: carico il file con la posizione nel kit
        // pos(?)
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator mg1(velocita_perc, q_goal);
        robot.control(mg1);

        Eigen::MatrixXd T_kit = readMeasuredPose(ALBERO);
        Eigen::Matrix3d R_kit = T_kit.block<3,3>(0,0);
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)+0.10),R_kit, 0, tempo); 
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)),R_kit, 0, tempo); 
        if (!gripper.move(gripper_state.max_width, 0.1)) {
          //std::cout << "Failed open." << std::endl;
          esito = -1;
        }
        move_ee(robot_ip,  Eigen::Vector3d(T_kit(0,3),T_kit(1,3),T_kit(2,3)+0.10),R_kit, 0, tempo); 
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
      }
    }


    if(esito == -1){
      fprintf(file0_, "NOK");
      fclose(file0_);
      return -1;
    }

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    fclose(file2);

    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }

  fprintf(file0_, "OK");
  fclose(file0_);

  fclose(file2);
  return 0;
}

Eigen::MatrixXd readMeasuredPose(int albero){
  FILE* file;
  if(albero == 1)
    file = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/graspMeasuredPoseAspirazione.txt", "r");
  else
    file = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/graspMeasuredPoseScarico.txt", "r");
  Eigen::MatrixXd P;
  char stringa[80];
  int res;
  int i = 0;
  int j = 0;
  res = fscanf(file,"%s",stringa);
  int numPti = stoi(stringa);
  res = fscanf(file,"%s",stringa);
  double tempo = stod(stringa);
  P.resize(numPti, 3);
  Eigen::Matrix3d Rd;

  res = fscanf(file,"%s",stringa);
  while(i < numPti && res != EOF) {
    //cout << stringa << endl;
    if(strcmp(stringa, "Rd") == 0){
      Rd = readMatrix(file);
      j = 0;
      i++;
    } else {
      P(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file,"%s",stringa);
  }

  Eigen::MatrixXd T(4,4);
  T.block<3,3>(0,0) = Rd;
  T(0,3) = P(0,0);
  T(1,3) = P(0,1);
  T(2,3) = P(0,2);
  std::cout << "\n\n\n\n\nT\n\n\n\n" << T << std::endl;
  fclose(file);
  return T;
}

int move_ee(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf){
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

