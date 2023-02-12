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
//tempo di fine
//const double t_f = 5.0;
//tempo di assestamento per essere sicuri che l'errore vada a zero
//const double t_ass = 0.1;//1.5;

double ForzaLim=0; // FORZA DI CONTATTO LIMITE PER LA DETECTION DELL'ALBERO DA PRENDERE
double tau_fl=0;   // COSTANTE DI TEMPO DEL GRADINO ESPONENZIALE UTILIZZATO PER AZZERARE LE q_dot
const int SEG_RIL=20; // SEGMENTO NEL QUALE AVVIENE LO SCAMBIO
bool IS_SHAFT_ROTATE_asp;
Eigen::Vector3d ffr1;

//variabili per il salvataggio su file
FILE* file0_;

int ALBERO;
bool grasp =false;

// *************** DICHIARAZIONE FUNZIONI
int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double ti, double tf);


using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << "<input file path> " << std::endl;
    file0_ = fopen("/home/labarea-franka/Desktop/CODICI/socket/resultFile/esito.txt", "w");
    fprintf(file0_, "NOK");
    fclose(file0_);
    return -1;
  }

  t_ass = 0.1;
  
  std::string robot_ip = argv[1];
  std::string filepath = argv[2];

  ALBERO = 1; //stoi(argv[4]);
  //ROTAZIONE ALBERI
  FILE* file_rot;
  file_rot = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/isRotate.txt", "r");
  char stringa1[80];
  fscanf(file_rot,"%s",stringa1);
  int val = std::stoi(stringa1);
  IS_SHAFT_ROTATE_asp = (val == 0) ? false : true;

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
std::cout << "TEMPO : " << tempo << std::endl;
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

  try{
ffr1 << 0,0,0; 

    int seg=1;
    int esito =2;

//int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, double tf, int seg);
    while(seg<=3){
      if(seg==1){
        esito = move(robot_ip,  Eigen::Vector3d(P(seg,0), P(seg,1),P(seg,2)),Rf_vec[seg], 0, tempo); 
        if(esito==-1)
          break;
    
      }else if(seg==2){    
        esito = move(robot_ip,  Eigen::Vector3d(P(seg,0), P(seg,1),P(seg,2)),Rf_vec[seg], 0, tempo); 
        //std::cout << "ESITO: " << esito << std::endl;
        std::cout << gripper_state.temperature << std::endl;
        if(esito==-1){
          break;
        }
        //if (!gripper.grasp(0.0080, 0.1, 100)) { //Valore di Grasp per il gripper di Michelangelo
        //if (!gripper.grasp(0.0418, 0.1, 100)) { //Valore di Grasp per il gripper di CRF
        if (!gripper.grasp(0.0140921, 0.1, 100)) { 
          //std::cout << "Failed to grasp object." << std::endl;
          //return -1;
          if (!gripper.move(gripper_state.max_width, 0.1)) {
            esito = -1;
            break;
          }
          esito = -1;
          break;
        }

        //Salvo la posizione di grasp dell'albero
        Eigen::Map<const Eigen::Matrix<double, 7, 1> > q_grasp(robot_state.q_d.data());
        std::vector<double> theta_grasp;
        theta_grasp = {q_grasp[0],q_grasp[1],q_grasp[2],q_grasp[3],q_grasp[4],q_grasp[5],q_grasp[6]};
        Eigen::MatrixXd F_grasp;
        F_grasp = fk(a, d, alpha, theta_grasp);
        FILE* file_pos_grasp;
        file_pos_grasp = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/graspMeasuredPoseAspirazione.txt", "w");
        fprintf(file_pos_grasp, "1 10\n%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n",
              F_grasp(0,3), F_grasp(1,3), F_grasp(2,3), F_grasp(0,0), F_grasp(0,1), F_grasp(0,2),
                                                        F_grasp(1,0), F_grasp(1,1), F_grasp(1,2),
                                                        F_grasp(2,0), F_grasp(2,1), F_grasp(2,2));
        fclose(file_pos_grasp);
        
        grasp=true;
        

      } else if(seg==3){ 
        //std::cout << "SONO IN SEG 3"  << std::endl;   
        esito = move(robot_ip,  Eigen::Vector3d(P(seg,0), P(seg,1),P(seg,2)),Rf_vec[seg], 0, tempo);   
        //std::cout << "ESITO seg 3: " << esito << std::endl; 
        if(esito==-1){
          break;
        }
      }
      seg++;
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

