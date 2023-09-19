#ifndef FSM_State_BalanceStandVBL_H
#define FSM_State_BalanceStandVBL_H

#include "FSM_State.h"
#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include <Controllers/BalanceController/BalanceControllerVBL.hpp>
#include <Controllers/BalanceController/ReferenceGRF.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "NN_two_legs_data.hpp"
#include <lcm-cpp.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <Utilities/Timer.h>

using namespace Eigen;
using namespace std;


template <typename T>
class FSM_State_BalanceStandVBL : public FSM_State<T>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStandVBL(ControlFSMData<T>* _controlFSMData);


  
  // Behavior to be carried out when entering a state
  void onEnter() override;

  // Run the normal behavior for the state
  void run();


  void publish_data_lcm();
  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();


  // Variable for angular steps 
  double roll_des,pitch_des,yaw_des;
  float j1 = 0.0;
  double j2 = 0.0;
  bool turn = false;

 
  //optima GRF
  double  fOpt_world[12];
  double  dfVBL[12];
  Eigen::VectorXd tempVector3; 
  



 private:

    lcm::LCM * _NNlcm;
    NN_two_legs_data NN_two_legs_da, NN_two_legs_da_publish;
    
  
    Eigen::MatrixXd s_LQR;
    ConvexMPCLocomotion* cMPCOld;
    BalanceControllerVBL balanceControllerVBL;
    ReferenceGRF refGRF;
   
    /* Actual Kinematics*/      
    Eigen::VectorXd x_COM_world;
    Eigen::VectorXd xdot_COM_world;            
    Eigen::VectorXd omega_b_world;
    Eigen::VectorXd omega_b_body; // new
    Eigen::VectorXd quat_b_world;
    Eigen::MatrixXd R_b_world;
    Eigen::MatrixXd p_feet; 

    /* Desired Kinematics */
    Eigen::VectorXd x_COM_world_desired;
    Eigen::VectorXd xdot_COM_world_desired;
    Eigen::VectorXd xddot_COM_world_desired;      
    Eigen::VectorXd omega_b_world_desired;
    Eigen::VectorXd omega_b_body_desired; // new       
    Eigen::VectorXd omegadot_b_world_desired;      
    Eigen::MatrixXd R_b_world_desired;
    Eigen::MatrixXd p_feet_desired; //new
		Eigen::VectorXd f_tot_Eig;

    /*Error coordinates */
    Eigen::VectorXd vbd_command_eigen;
    Eigen::VectorXd orientation_error; 
    Eigen::VectorXd error_x_lin; // new
    Eigen::VectorXd error_dx_lin; // new
    Eigen::VectorXd error_R_lin; // new
    Eigen::VectorXd error_omega_lin; // new

    void get_desired_state();
    void get_model_dynamics();
    void get_foot_locations();
    void liftLeg(int leg, Vec3<T> q, Vec3<T> qd);
    void rpyToR(Mat3<float> &R, double* rpy_in);
    void eulerToQuat(double* rpy_in, double* quat_in);
    void quatToEuler(double* quat_in, double* rpy_in);

   
    void matrixExpOmegaCross(const Eigen::VectorXd & omega, Eigen::MatrixXd & R);
    void matrixLogRot(const Eigen::MatrixXd & R, Eigen::VectorXd & omega);
    void crossMatrix(Eigen::MatrixXd &R, const Eigen::VectorXd &omega);
    void inverseCrossMatrix(const Eigen::MatrixXd &R, Eigen::VectorXd &omega); 
    void quaternion_to_rotationMatrix(Eigen::MatrixXd &R, Eigen::VectorXd &quat);  

    void rpyToR(Eigen::MatrixXd &R, double* rpy_in);


    void calc_linear_error(Eigen::VectorXd x_COM_world,Eigen::VectorXd x_COM_world_desired,Eigen::VectorXd xdot_COM_world,Eigen::VectorXd xdot_COM_world_desired,Eigen::MatrixXd R_b_world,Eigen::MatrixXd R_b_world_desired,Eigen::VectorXd omega_b_body, Eigen::VectorXd omega_b_body_desired,Eigen::VectorXd &s);


    void run_NN(Eigen::MatrixXd input,double* ex);
  
 

   // Support variables
    int iter = 0;
    bool firstRun = true;
    float start; 
    bool firstInside = true;
    Timer change;
    // Actual state of the body
    double pFeet[12], p_act[3], v_act[3], quat_act[4], rpy_act[3], se_xfb[13], pFeet_world[12], p_body[3],p_COM[3]; 
 
    // Desired state of the body
    double rpy[3],pFeet_des[12], p_des[3], v_des[3],rpy_des[3], omegaDes[3];
    double pweight, convert = 3.14159/180;
    double Ig_in[3], mass_in;

    FloatingBaseModel<T> model;
    FBModelState<T> state;
    Vec3<T> c_body, c_world;
    Mat18<float> H;
    DVec<T> G_ff;


    // Feet relative to COM
    Vec3<T> pFeetVec;
    Vec3<T> pFeetVecBody;

    Vec3<T> pFeetVec_debug;
    Vec3<T> pFeetVecBody_debug;
  

     // LQR Weights
     double x_weights[3], xdot_weights[3], R_weights[3], omega_weights[3];
     double alpha_control, beta_control;

     // Joint positions for legs not in contact
     Vec3<T> q_lift_leg, q_lift_leg_1,qd_lift_leg;
     Mat3<T> kpMat, kdMat;

    // Contact Data
     double minForce, maxForce, mu_ctrl = 0.6;
     double minForces[4], maxForces[4];
     double contactStateScheduled[4] = {1.0, 1.0, 1.0, 1.0};
 
 
     // Control Input
     double f_ref_z[4], f_ref_world[12], fOpt[12],fOpt1[12],df[6],f_tot[12],xOpt[12];
     Vec4<T> conPhase;

     // Leg Impedance Control
     Vec3<double> impedance_kp;
     Vec3<double> impedance_kd;
     bool ESTOP;
  

     // Check to see if desired position has changed
     double p_des_prev[2] = {0.0,0.0};

     // Account for non-zero initial yaw
     double ini_yaw;
     Mat3<float> rBody_yaw; 

     // Compare expected joint torques to actual leg controller
    double tauOpt[3], tauEst[3];
    Mat3<T> Jleg;
    
    Eigen::MatrixXd hid1; 
    Eigen::MatrixXd hid2;
    Eigen::MatrixXd ex1;
    Eigen::MatrixXd tmp;

		
   template<typename M>
   M load_csv (const std::string & path) 
   {
       std::ifstream indata;
       indata.open(path);
       std::string line;
       std::vector<double> values;
       uint rows = 0;
       while (std::getline(indata, line)) 
       {
           std::stringstream lineStream(line);
           std::string cell;
           while (std::getline(lineStream, cell, ',')) 
          {
             values.push_back(std::stod(cell));
          }
           ++rows;
       }

   return Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime,  RowMajor>>(values.data(), rows, values.size()/rows);
   }


	

 // ------------------ 	NEURAL NETWORK OPTIMAL WEIGHTS  ----------------------------------------------------


   Eigen::MatrixXd W1 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Desktop/Two_legs_balancing_RAL/Weights/W1.csv");
   Eigen::MatrixXd W2 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Desktop/Two_legs_balancing_RAL/Weights/W2.csv");
   Eigen::MatrixXd W3 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Desktop/Two_legs_balancing_RAL/Weights/W3.csv");

   


   void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd &source, int nRows, int nCols);                  
   void copy_Eigen_to_double(double* target, Eigen::VectorXd &source, int length);
   void copy_Array_to_Eigen(Eigen::VectorXd &target, double* source, int len, int startIndex);
   void copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t* source, int len);



};

#endif  // FSM_State_BalanceStandVBL_H
