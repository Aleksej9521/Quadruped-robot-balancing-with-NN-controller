#include "FSM_State_BalanceStandVBL.h"
#include <iostream>
#include<fstream>
#include<stdio.h>
#include <Utilities/Timer.h>
#include <chrono>
#include <stdlib.h> 


using namespace std;
using namespace std::chrono;

template <typename T>
FSM_State_BalanceStandVBL<T>::FSM_State_BalanceStandVBL(ControlFSMData<T>* _controlFSMData): FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STANDVBL,"BALANCE_STANDVBL"){
  
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
  this->checkSafeOrientation = false;
 
  model = _controlFSMData->_quadruped->buildModel();
  

  _NNlcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");

  

  
   if (_NNlcm->good())
   {
      printf("LCM IN NN CONTROL INITIALIZED\n");
   }
   else
   {
      printf("LCM IN NN CONTROLLER FAILED\n");
      exit(-1);
   }

  
}




template <typename T>
void FSM_State_BalanceStandVBL<T>::onEnter() {
    
  // Default is to not transition
  this->nextStateName = this->stateName;

  this->transitionData.zero();
  
  
   iter = 0;  

   x_COM_world.resize(3,1);
   xdot_COM_world.resize(3,1);            
   omega_b_world.resize(3,1);
   omega_b_body.resize(3,1);
   quat_b_world.resize(4,1);
   R_b_world.resize(3,3);
  
   /* Desired Kinematics */
   x_COM_world_desired.resize(3,1);
   xdot_COM_world_desired.resize(3,1);
   xddot_COM_world_desired.resize(3,1);      
   omega_b_world_desired.resize(3,1);
   omegadot_b_world_desired.resize(3,1);      
   R_b_world_desired.resize(3,3);

   
    error_x_lin.setZero(3,1); 
    error_dx_lin.setZero(3,1);
    error_R_lin.setZero(3,1);
    error_omega_lin.setZero(3,1);
      
  
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::run() {
     
      iter++; 
   
   if(firstInside == true)
   {
       start  = change.getSeconds();
       firstInside = false;
   }
  
  
    
   for (int i = 0; i < 3; i++) {
        x_weights[i] = this->_data->userParameters->Q_pos[i];
        xdot_weights[i] = this->_data->userParameters->Q_vel[i];
        R_weights[i] = this->_data->userParameters->Q_ori[i];
        omega_weights[i] = this->_data->userParameters->Q_ang[i];
   }
   
   alpha_control = this->_data->userParameters->R_control;
   beta_control = this->_data->userParameters->R_prev;
   balanceControllerVBL.set_LQR_weights(x_weights,xdot_weights,R_weights,omega_weights,alpha_control,beta_control);
   
   
   // Get state values
    for (int i = 0; i < 4; i++)
    {
        quat_act[i] = (double)this->_data->_stateEstimator->getResult().orientation(i);
    }

  // Get current state from state estimator
  for (int i = 0; i < 3; i++) {
    p_act[i] = (double)this->_data->_stateEstimator->getResult().position(i);
    se_xfb[4 + i] = (double)this->_data->_stateEstimator->getResult().position(i);
    se_xfb[7 + i] = (double)this->_data->_stateEstimator->getResult().omegaBody(i);
    se_xfb[10 + i] = (double)this->_data->_stateEstimator->getResult().vBody(i);
  }

  
  
  // Convert quaternions to RPY 
  quatToEuler(quat_act, rpy_act);

  // Account for initial yaw
  if (iter < 10)
    ini_yaw = rpy_act[2];
  rpy_act[2] -= ini_yaw;
  
  // Convert back to quaternions w/ initial yaw accounted for
  eulerToQuat(rpy_act, quat_act);
  for (int i = 0; i < 4; i++) 
  {
    se_xfb[i] = quat_act[i];
  }
  
    
  // Get the position of the COM in world frame & generalized gravity vector
  get_model_dynamics();
  
  // Use the COM pos instead of the body pos for balance control
  for (int i = 0; i < 3; i++) {
    p_body[i] = p_act[i];
    se_xfb[i+4] = p_COM[i]; 
    p_act[i] = p_COM[i];
  }

    auto* ball2 = this->_data->visualizationData->addSphere();
    ball2->radius = 0.01;
    ball2->position = {p_act[0],p_act[1],0.02};// Set Control Parameters
    ball2->color = {0.0, 1.0, 0.0, 0.5};
    
  // Get desired state from gamepad controls
  get_desired_state(); 

  // Get the foot locations relative to COM
  get_foot_locations();
  
   
    minForce = 5;
    maxForce = 70.0; // mass_in*9.81*1.6;
    for (int leg = 0; leg < 4; leg++) {
        minForces[leg] = contactStateScheduled[leg] * minForce;
        maxForces[leg] = contactStateScheduled[leg] * maxForce;
    }
        
    
  // Compute Reference Control Inputs - only if desired xy pos has changed
    if (this->_data->userParameters->stance_legs == 4 || (this->_data->userParameters->stance_legs == 2 && firstRun == true)   || (this->_data->userParameters->use_twolegs == 0 && firstRun == true))
    {
        
        refGRF.set_alpha_control(0.01);
        refGRF.set_mass(mass_in);
        refGRF.SetContactData(contactStateScheduled, minForces, maxForces);
        refGRF.updateProblemData(pFeet_des, p_des);
        refGRF.solveQP_nonThreaded(f_ref_z);
        for (int leg = 0; leg < 4; leg++)
        f_ref_world[3*leg+2] = f_ref_z[leg];
        balanceControllerVBL.set_reference_GRF(f_ref_world); 
 
        if((f_ref_world[5] >= 43.9961 && f_ref_world[8] >= 43.9961))
            firstRun = false;
        
        
    }
    
   //PARTE RETE NEURALE
  if(this->_data->userParameters->use_twolegs > 0)
  {
   

   copy_Array_to_Eigen(quat_b_world, se_xfb, 4, 0);
   copy_Array_to_Eigen(x_COM_world, se_xfb, 3,4);
   copy_Array_to_Eigen(omega_b_world, se_xfb, 3,7);   
   copy_Array_to_Eigen(xdot_COM_world, se_xfb, 3,10);
   
   quaternion_to_rotationMatrix(R_b_world,quat_b_world);
        
   omega_b_body = R_b_world.transpose()*omega_b_world; 
   
   //salvataggio traiettoria desiderata
   x_COM_world_desired << p_des[0], p_des[1], p_des[2];
   rpyToR(R_b_world_desired, rpy);
   omega_b_world_desired << omegaDes[0], omegaDes[1], omegaDes[2];   
   
   omega_b_body_desired = R_b_world_desired.transpose()*omega_b_world_desired;
   
   xdot_COM_world_desired << v_des[0], v_des[1], v_des[2];   
     
   Eigen::VectorXd prov;

   double df_sel[6];
   
   
   prov.resize(12,1);
   calc_linear_error(x_COM_world,x_COM_world_desired,xdot_COM_world,xdot_COM_world_desired,R_b_world,R_b_world_desired,omega_b_body,omega_b_body_desired,prov); 
   Eigen::Map<Eigen::MatrixXd> M(prov.data(), 12, 1);
   Eigen::MatrixXd M2(M);
        
   
   run_NN(M2,df);
            
   
   
   for (int i = 0; i<6;i++)
   {       
       df_sel[i] =  df[i];
   }
                
            
   int j = 0;
   for(int i=0;i<12;i++)
   {
        if(i==0 || i == 1 || i ==2 || i==9 || i==10 || i==11)
        {
            f_tot[i] = 0;
        }
        else
        {
            f_tot[i] = f_ref_world[i] + df_sel[j];
            j++;
        }
    }
        
    
    f_tot_Eig.resize(12,1);
    copy_Array_to_Eigen(f_tot_Eig,f_tot,12,0);

            
        
    //World2Body        
    for(int i = 0; i < 4; i++)
    {
        tempVector3 = -R_b_world.transpose()*f_tot_Eig.segment(3*i,3);
        fOpt[3*i] = tempVector3(0);
        fOpt[3*i+1] = tempVector3(1);
        fOpt[3*i+2] = tempVector3(2);
    }

   
   //incapsulamento per salvataggio con lcm 
    
    double df_lcm[6];
    double p_des_lcm[3]; 
    double s_lcm[12];
    
    copy_Eigen_to_double(s_lcm,prov,12);
   
 
    for(int i = 0; i < 6; i++)
    {
      NN_two_legs_da.df[i] = df_sel[i];
    }
   
    for(int i = 0; i < 12; i++)
    {
       
      NN_two_legs_da.s[i] = s_lcm[i];
      NN_two_legs_da.p_feet_in[i] = pFeet[i];
      NN_two_legs_da.p_feet_desired_in[i] = pFeet_des[i];  
   
    }
     
    for(int i = 0; i < 3; i++)
    {
      NN_two_legs_da.rpy[i] = rpy[i];
      NN_two_legs_da.p_act[i] = p_act[i];
      NN_two_legs_da.rpy_act[i] = rpy_act[i];
    }
         
     _NNlcm->publish("NN_two_legs_da", &NN_two_legs_da);
    
    
  }
  
  else
  {

    balanceControllerVBL.set_friction(mu_ctrl);
    balanceControllerVBL.set_mass(mass_in);
    balanceControllerVBL.set_inertia(0.025, 0.15, 0.18);
         
             
    
    // Solve Balance control QP
    rpy_des[0] = 0;
    rpy_des[1] = 0;
    rpy_des[2] = 0;
    
    balanceControllerVBL.set_desiredTrajectoryData(rpy_des, p_des, omegaDes, v_des);
    balanceControllerVBL.SetContactData(contactStateScheduled, minForces, maxForces);
    
//     printf("Roll des: %f\n\n",rpy[0]);
//     printf("Pitch des: %f\n\n",rpy[1]);
//     printf("Yaw des: %f\n\n",rpy[2]);
//     
    balanceControllerVBL.updateProblemData(se_xfb, pFeet, pFeet_des, rpy, rpy_act,this->_data->userParameters->stance_legs,firstRun);
        
    balanceControllerVBL.solveQP_nonThreaded(fOpt,dfVBL);
    balanceControllerVBL.publish_data_lcm();
  }
 
  
  
  // Remove impedance control for all joints
  impedance_kp << 0.0, 0.0, 0.0;
  impedance_kd << 0.0, 0.0, 0.0;

  // Check for emergency stop if orientation error too large
  for(int i = 0; i < 3; i++){
    if(fabs(rpy[i]-rpy_act[i])>0.55)
      ESTOP = true;
  }

  // Feed forward forces for legs in contact with the ground & PD control for legs not in contact
  qd_lift_leg << 0.0, 0.0, 0.0;
  if(ESTOP){
    this->jointFeedForwardTorques = Mat34<float>::Zero();   // feed forward joint torques
    this->footFeedForwardForces = Mat34<float>::Zero();     // feedforward forces at the feet
  } else{
  for (int leg = 0; leg < 4; leg++) {
      
       
    if (contactStateScheduled[leg] > 0.0)
    {
        this->footFeedForwardForces.col(leg) << (T)fOpt[leg * 3],(T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];
        
        this->jointFeedForwardTorques.col(leg) << (T)G_ff[6+3*leg], (T)G_ff[6+3*leg+1], G_ff[6+3*leg+2];
        conPhase[leg] = 0.5;
    }

    else 
    {
       this->liftLeg(leg, q_lift_leg, qd_lift_leg);           
       conPhase[leg] = 0.0;
    }
  
     
  // Set the contact phase for the state estimator
  this->_data->_stateEstimator->setContactPhase(conPhase);

  // Update previous desired posiion
  for (int i = 0; i < 2; i++)
    p_des_prev[i] = p_des[i];

  // Send commands to leg controller
  for (int leg = 0; leg < 4; leg++) {

    // Force and Joint Torque control
    this->_data->_legController->commands[leg].forceFeedForward = this->footFeedForwardForces.col(leg);
    this->_data->_legController->commands[leg].tauFeedForward = this->jointFeedForwardTorques.col(leg);
  }

    
  
  // Compute expected torque for leg 1 to leg controller command
  computeLegJacobianAndPosition(**&this->_data->_quadruped, this->_data->_legController->datas[1].q, &Jleg, (Vec3<T>*)nullptr, 1);
  tauOpt[0] = Jleg(0,0)*fOpt[3]+Jleg(1,0)*fOpt[3+1]+Jleg(2,0)*fOpt[3+2]+G_ff[6+3];
  tauOpt[1] = Jleg(0,1)*fOpt[3]+Jleg(1,1)*fOpt[3+1]+Jleg(2,1)*fOpt[3+2]+G_ff[6+3+1];
  tauOpt[2] = Jleg(0,2)*fOpt[3]+Jleg(1,2)*fOpt[3+1]+Jleg(2,2)*fOpt[3+2]+G_ff[6+3+2];

  for (int i = 0; i < 3; i++)
    tauEst[i] = this->_data->_legController->datas[1].tauEstimate[i];
 }
 
  }

}
       

/**
 * Uses quadruped model to find CoM position relative to body position
 * and compute generalized gravity vector
 */

template <typename T>
void FSM_State_BalanceStandVBL<T>::calc_linear_error(Eigen::VectorXd x_COM_world,Eigen::VectorXd x_COM_world_desired,Eigen::VectorXd xdot_COM_world,Eigen::VectorXd xdot_COM_world_desired,Eigen::MatrixXd R_b_world,Eigen::MatrixXd R_b_world_desired,Eigen::VectorXd omega_b_body, Eigen::VectorXd omega_b_body_desired,Eigen::VectorXd &s)
{

  
  // Linear error for LQR
  error_x_lin = x_COM_world - x_COM_world_desired;
  error_dx_lin = xdot_COM_world - xdot_COM_world_desired;
 
  inverseCrossMatrix(0.5*(R_b_world_desired.transpose()*R_b_world-R_b_world.transpose()*R_b_world_desired),error_R_lin);
  
  error_omega_lin = omega_b_body - R_b_world.transpose()*R_b_world_desired*omega_b_body_desired;
     
  
  s.resize(12,1);
  s << error_x_lin(0),error_x_lin(1),error_x_lin(2),error_dx_lin(0),error_dx_lin(1),error_dx_lin(2),error_R_lin(0),error_R_lin(1),error_R_lin(2),error_omega_lin(0),error_omega_lin(1),error_omega_lin(2);
    
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::run_NN(Eigen::MatrixXd input,double* ex)
{
 
  
    hid1 = (W1*input).array().tanh();
    hid2 = (W2*hid1).array().tanh();
    ex1 = W3*hid2;
    
    Eigen::Map<Eigen::VectorXd> M(ex1.data(), 6, 1);
    Eigen::VectorXd M2(M);
    
    copy_Eigen_to_double(ex,M2,6);
    
}





template <typename T>
void FSM_State_BalanceStandVBL<T>::get_model_dynamics() {

    
    
   for (int i = 0; i < 3; i++){
    state.bodyOrientation[i] = quat_act[i];
    state.bodyPosition[i] = this->_data->_stateEstimator->getResult().position(i);
    state.bodyVelocity[i] = this->_data->_stateEstimator->getResult().omegaBody(i);
    state.bodyVelocity[i+3] = this->_data->_stateEstimator->getResult().vBody(i);
  }
  
  
  state.bodyOrientation[3] = quat_act[3];
  state.q.setZero(12);
  state.qd.setZero(12);
  
  for (int i = 0; i < 4; ++i) {
   state.q(3*i+0) = this->_data->_legController->datas[i].q[0];
   state.q(3*i+1) = this->_data->_legController->datas[i].q[1];
   state.q(3*i+2) = this->_data->_legController->datas[i].q[2];
   state.qd(3*i+0)= this->_data->_legController->datas[i].qd[0];
   state.qd(3*i+1)= this->_data->_legController->datas[i].qd[1];
   state.qd(3*i+2)= this->_data->_legController->datas[i].qd[2];
  }
  
  
  
  model.setState(state);
   
  H = model.massMatrix();
  G_ff = model.generalizedGravityForce();

  // CoM relative to body frame
  mass_in = H(3,3);
  c_body[0] = H(2,4)/mass_in;
  c_body[1] = H(0,5)/mass_in;
  c_body[2] = H(1,3)/mass_in;

  // CoM relative to world frame
  rpyToR(rBody_yaw, rpy_act);
  c_world = rBody_yaw * c_body;

  // Position of CoM in world frame
  for (int i = 0; i < 3; i++)
    p_COM[i] = p_act[i]+c_world[i]; // assumes C is expressed in world frame

}



template <typename T>
void FSM_State_BalanceStandVBL<T>::get_desired_state() {
  // Nominal state
  
    p_des[2] = 0.225;
   
  
   double elaps_time = change.getSeconds() - start;
    
   if(this->_data->userParameters->step_roll > 0)
   {
    if(elaps_time >2.0)
    {
        if(turn == false)
        {
            j1 = j1 + 0.1;
        }
        if(turn == true)
        {
            j1 = j1 - 0.1;
        }
        
        start  = change.getSeconds();
        
        if(j1 <= -0.4) 
        {
            turn = false;
        }
        
        if(j1 >= 0.6) //j1 >= 0.7 //0.7 massimo VBL 
        {   
            turn = true;
        }
        
        roll_des = j1;
       
       
    }
    
   } 
   
   if(this->_data->userParameters->step_pitch > 0)
   {
    if(elaps_time >=2.0)
    {
        if(turn == false)
        {
            j2 = j2 + 0.1;
        }
        else
        {
            j2 = j2 - 0.1;
        }
        
        pitch_des = j2;
        start  = change.getSeconds();
        
        if(j2 == 0.4)
        {   
            turn = true;
        }
        
        if(j2 == -0.5)
        {
            turn = false;
        }
        
        
    }

   }
   
   if(this->_data->userParameters->step_yaw > 0)
   {
    if(elaps_time >=2.0)
    {
        if(turn == false)
        {
            j1 = j1 + 0.1;
        }
        if(turn == true)
        {
            j1 = j1 - 0.1;
        }
        
          start  = change.getSeconds();
          
        if(j1 <= -0.7)  
        {
            turn = false;
        }
        
        if(j1 >= 0.2)
        {   
            turn = true;
        }
        
        yaw_des = j1;
       
       
    }
   }
    
    rpy[0] = roll_des;
    rpy[1] = pitch_des;
    rpy[2] = yaw_des;
    
    //FOR DEBUG
    printf("-----VALORI ANGOLI-----");
    printf("r_des_in:%f ° \n\n ",rpy[0]);
    printf("p_des_in:%f ° \n\n ",rpy[1]);
    printf("y_des_in:%f ° \n\n ",rpy[2]);
      
      
      
      
      
    for (int i = 0; i < 3; i++) 
    {
      omegaDes[i] = 0.0;
      v_des[i] = 0.0;
    }

  // Lift legs
    if (this->_data->userParameters->stance_legs == 2) 
    { // two legs
        
        pweight = 0.5 + 0.075 * this->_data->_desiredStateCommand->data.stateDes[6];
        p_des[0] = (pweight * pFeet_world[3*1] + (1 - pweight) * pFeet_world[3*2]);
        p_des[1] = pweight * pFeet_world[3*1+1] + (1 - pweight) * pFeet_world[3*2+1];
        
        contactStateScheduled[0] = 0.0;
        contactStateScheduled[3] = 0.0;
        
        q_lift_leg << 0., -1.45, 2.9; //lift leg position 
    
    } 
    else if(this->_data->userParameters->stance_legs == 3.0)
    { // three legs
        pweight = 0.425;
        
    
        // Alzo zampa anteriore sinistra !
        p_des[0] = pweight * pFeet_world[3] + (1 - pweight) * pFeet_world[6];
        p_des[1] = pweight * pFeet_world[4] + (1 - pweight) * pFeet_world[7];
    
   
        if (contactStateScheduled[1] >= 0.2)
        {
        contactStateScheduled[1] -= 0.0005;
        }
        else
        {
        contactStateScheduled[1] = 0.0;
        } 
    
  
    }
  else  if (this->_data->userParameters->stance_legs == 4)
  { // four leg
        pweight = 0.5 + 0.075 * this->_data->_desiredStateCommand->data.stateDes[6];
        p_des[0] = pweight * pFeet_world[3*1] + (1 - pweight) * pFeet_world[3*2];
        pweight = 0.5 + 0.05 * this->_data->_desiredStateCommand->data.stateDes[7];
        p_des[1] = pweight * pFeet_world[3*1+1] + (1 - pweight) * pFeet_world[3*2+1];
  }

  
}

/**
 * Use data from leg controller and state estimator to compute positions
 * of the feet relative to the CoM
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::get_foot_locations() 
{

  for (int leg = 0; leg < 4; leg++)
  {
    
    // Compute vector from hip to foot (body coords)
    computeLegJacobianAndPosition(**&this->_data->_quadruped, this->_data->_legController->datas[leg].q,
                                  &this->_data->_legController->datas[leg].J, &pFeetVec, leg);
    
    // Compute vector from origin of body frame to foot (world coords)
    pFeetVecBody = rBody_yaw * (this->_data->_quadruped->getHipLocation(leg) + pFeetVec);
    

    // Compute vector from COM to foot (world coords)
    pFeet[leg * 3] = (double)pFeetVecBody[0]-c_world[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecBody[1]-c_world[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecBody[2]-c_world[2];
  
    
   
  if(firstRun == true)
  {
    // Compute vector from desired COM to foot (world coords)
    pFeet_des[leg * 3] = (double)pFeetVecBody[0]-c_world[0]+p_act[0]-p_des[0];
    pFeet_des[leg * 3 + 1] = (double)pFeetVecBody[1]-c_world[1]+p_act[1]-p_des[1];
    pFeet_des[leg * 3 + 2] = (double)pFeetVecBody[2]-c_world[2]+p_act[2]-p_des[2];
 }

        // Locations of the feet in world coordinates
        pFeet_world[leg * 3] = (double)p_act[0]+pFeet[leg * 3];
        pFeet_world[leg * 3 + 1] = (double)p_act[1]+pFeet[leg * 3 + 1];
        pFeet_world[leg * 3 + 2] = (double)p_act[2]+pFeet[leg * 3 + 2];

    }
}


template <typename T>
void FSM_State_BalanceStandVBL<T>::copy_Eigen_to_double(double* target, Eigen::VectorXd &source, int length)
{
   for(int i = 0; i < length; i++)
   {
      target[i] = source(i);
   }
    
}


/**
 * Convert euler angles to rotation matrix
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::rpyToR(Mat3<float> &R, double* rpy_in) {

   Mat3<float> Rz, Ry, Rx;

   Rz.setIdentity();
   Ry.setIdentity();
   Rx.setIdentity();

   Rz(0,0) = cos(rpy_in[2]);
   Rz(0,1) = -sin(rpy_in[2]);
   Rz(1,0) = sin(rpy_in[2]);
   Rz(1,1) = cos(rpy_in[2]);

   Ry(0,0) = cos(rpy_in[1]);
   Ry(0,2) = sin(rpy_in[1]);
   Ry(2,0) = -sin(rpy_in[1]);
   Ry(2,2) = cos(rpy_in[1]);

   Rx(1,1) = cos(rpy_in[0]);
   Rx(1,2) = -sin(rpy_in[0]);
   Rx(2,1) = sin(rpy_in[0]);
   Rx(2,2) = cos(rpy_in[0]);

   R = Rz*Ry*Rx;

 }

/**
 * Convert euler angles to quaternions
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::eulerToQuat(double* rpy_in, double* quat_in) {

  quat_in[0] = cos(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5) + sin(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5);
  quat_in[1] = cos(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5) - sin(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5);
  quat_in[2] = sin(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5) + cos(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5);
  quat_in[3] = sin(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5) - cos(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5);

}

/**
 * Convert quaternions to euler angles
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::quatToEuler(double* quat_in, double* rpy_in) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (quat_in[0] * quat_in[1] + quat_in[2] * quat_in[3]);
  double cosr_cosp = +1.0 - 2.0 * (quat_in[1] * quat_in[1] + quat_in[2] * quat_in[2]);
  rpy_in[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (quat_in[0] * quat_in[2] - quat_in[3] * quat_in[1]);
  if (fabs(sinp) >= 1)
    rpy_in[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    rpy_in[1] = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (quat_in[0] * quat_in[3] + quat_in[1] * quat_in[2]);
  double cosy_cosp = +1.0 - 2.0 * (quat_in[2] * quat_in[2] + quat_in[3] * quat_in[3]);  
  rpy_in[2] = atan2(siny_cosp, cosy_cosp);
}


template <typename T>
void FSM_State_BalanceStandVBL<T>::copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd &source, int nRows, int nCols )
{
   int count = 0;

   // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows) 
   for(int i = 0; i < nRows; i++)
   {      
      for (int j = 0; j < nCols; j++)
      {
         target[count] = source(i,j);
         count++;
      }
   }  
}




template <typename T>
void FSM_State_BalanceStandVBL<T>::copy_Array_to_Eigen(Eigen::VectorXd &target, double* source, int len, int startIndex)
{
   for(int i = 0; i < len; i++)
   {
      target(i) = source[i+startIndex];
   }
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t* source, int len)
{
   for(int i = 0; i < len; i++)
   {
      target(i) = source[i];
   }
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::inverseCrossMatrix(const Eigen::MatrixXd &R, Eigen::VectorXd &omega)
{
  omega(0) = R(2,1);
  omega(1) = R(0,2);
  omega(2) = R(1,0);
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::quaternion_to_rotationMatrix(Eigen::MatrixXd &R, Eigen::VectorXd &quat)
{
   //wikipedia
   R(0,0) = 1-2*quat(2)*quat(2)-2*quat(3)*quat(3);
   R(0,1) = 2*quat(1)*quat(2)-2*quat(0)*quat(3);
   R(0,2) = 2*quat(1)*quat(3)+2*quat(0)*quat(2);
   R(1,0) = 2*quat(1)*quat(2)+2*quat(0)*quat(3);
   R(1,1) = 1-2*quat(1)*quat(1)-2*quat(3)*quat(3);
   R(1,2) = 2*quat(2)*quat(3)-2*quat(1)*quat(0);
   R(2,0) = 2*quat(1)*quat(3)-2*quat(2)*quat(0);
   R(2,1) = 2*quat(2)*quat(3)+2*quat(1)*quat(0);
   R(2,2) = 1-2*quat(1)*quat(1)-2*quat(2)*quat(2);
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::rpyToR(Eigen::MatrixXd &R, double* rpy_in)
{
   Eigen::Matrix3d Rz, Ry, Rx;

   Rz.setIdentity();
   Ry.setIdentity();
   Rx.setIdentity();

   Rz(0,0) = cos(rpy_in[2]);
   Rz(0,1) = -sin(rpy_in[2]);
   Rz(1,0) = sin(rpy_in[2]);
   Rz(1,1) = cos(rpy_in[2]);

   Ry(0,0) = cos(rpy_in[1]);
   Ry(0,2) = sin(rpy_in[1]);
   Ry(2,0) = -sin(rpy_in[1]);
   Ry(2,2) = cos(rpy_in[1]);

   Rx(1,1) = cos(rpy_in[0]);
   Rx(1,2) = -sin(rpy_in[0]);
   Rx(2,1) = sin(rpy_in[0]);
   Rx(2,2) = cos(rpy_in[0]);

   R = Rz*Ry*Rx;
}

template <typename T>
void FSM_State_BalanceStandVBL<T>::liftLeg(int leg, Vec3<T> qDes, Vec3<T> qdDes)
{

  kpMat << 15, 0, 0, 0, 15, 0, 0, 0, 15;
  kdMat << 2, 0, 0, 0, 2, 0, 0, 0, 2;

  this->_data->_legController->commands[leg].kpJoint = kpMat;
  this->_data->_legController->commands[leg].kdJoint = kdMat;

  this->_data->_legController->commands[leg].qDes = qDes;
  this->_data->_legController->commands[leg].qdDes = qdDes;
}






template <typename T>
void saveData(std::string fileName, Eigen::Matrix<fpt,Dynamic,Dynamic> matrix)
{

    const static IOFormat CSVFormat(Eigen::FullPrecision, DontAlignCols, ", ", "\n");
 
    ofstream file;
    file.open(fileName,std::ios::app);
    
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file << "\n";
        file.close();
    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BalanceStandVBL<T>::checkTransition() {
  // Get the next state
  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_BALANCE_STAND_VBL:
      break;

    case K_LOCOMOTION:
      // Requested change to balance stand
      this->nextStateName = FSM_StateName::LOCOMOTION;

      // Transition instantaneously to locomotion state on request
      this->transitionDuration = 0.0;

      // Set the next gait in the scheduler to
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

   
    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;


    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_BALANCE_STAND_VBL << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_BalanceStandVBL<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
   
    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;
      
    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceStandVBL<T>::onExit(){
}





template class FSM_State_BalanceStandVBL<float>;
