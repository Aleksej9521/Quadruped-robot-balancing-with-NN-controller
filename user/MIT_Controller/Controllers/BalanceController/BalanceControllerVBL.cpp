
/*
+
+References:
+   [R1] M. Focchi, A. del Prete, I. Havoutis, R. Featherstone, D. G. Caldwell, and C. Semini. High-slope terrain
+   locomotion for torque-controlled quadruped robots. Autonomous Robots, 2016.
+
+   [R2] R. M. Murray, S. S. Sastry, and L. Zexiang. A Mathematical Introduction to Robotic Manipulation. CRC
+   Press, Inc., Boca Raton, FL, USA, 1st edition, 1994.
+
+Cheetah-3-Documentation-Control:
+   [C1] BalanceControllerVBL.pdf
+
+   qpOASES variables are terminated with _qpOASES
+*/


#include "BalanceControllerVBL.hpp"
#include<iomanip>
// #include "Sim_Utils.h"
using namespace std;

BalanceControllerVBL::BalanceControllerVBL() :  QProblemObj_qpOASES(vblNUM_VARIABLES_QP,vblNUM_CONSTRAINTS_QP)
{
   // *!! It seems like this next block of 5 lines does not actually do anything, 
   // I had to set print level in the OptimizationThread class
   Options options;
   options.printLevel = PL_NONE;
   QProblemObj_qpOASES.setOptions( options );    
   QProblemObj_qpOASES.setPrintLevel(PL_NONE);

   QPFinished = false;


   lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
   if (lcm->good())
   {
      printf("LCM IN BALANCE CONTROL INITIALIZED\n");
   }
   else
   {
      printf("LCM IN BALANCE CONTROLLER FAILED\n");
      exit(-1);
   }


   // Eigen QP matrices
   H_eigen.resize(vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP);
   A_eigen.resize(vblNUM_CONSTRAINTS_QP,vblNUM_VARIABLES_QP);
   g_eigen.resize(vblNUM_VARIABLES_QP,1);
   xOpt_eigen.resize(vblNUM_VARIABLES_QP,1);
   xOpt_combined.resize(vblNUM_VARIABLES_QP,1);
   yOpt_eigen.resize(vblNUM_VARIABLES_QP+vblNUM_CONSTRAINTS_QP,1);

   mass = 41;
   inertia = 0.01;
  
   /* Model and World parameters and force limits */
   Ig.resize(3,3);
   Ig << .35 , 0 ,0,  
         0   , 2.1 , 0,
         0   , 0   , 2.1;
   gravity.resize(3,1);

   direction_normal_flatGround.resize(3,1);
   direction_tangential_flatGround.resize(3,1);

   /* Initialize to all feet on the ground */
   contact_state.resize(4,1);
   contact_state << 1,1,1,1;

   minNormalForces_feet.resize(4,1);
   maxNormalForces_feet.resize(4,1);

   /* Actual Kinematics*/
   x_COM_world.resize(3,1);
   xdot_COM_world.resize(3,1);            
   omega_b_world.resize(3,1);
   omega_b_body.resize(3,1);
   quat_b_world.resize(4,1);
   R_b_world.resize(3,3);
   p_feet.resize(3,4);   

   /* Desired Kinematics */
   x_COM_world_desired.resize(3,1);
   xdot_COM_world_desired.resize(3,1);
   xddot_COM_world_desired.resize(3,1);      
   omega_b_world_desired.resize(3,1);
   omega_b_body_desired.resize(3,1); 
   omega_b_body_desired_real.resize(3,1);
   omegadot_b_world_desired.resize(3,1);      
   R_b_world_desired.resize(3,3);
   R_b_world_desired_real.resize(3,3);
   orientation_error.resize(3,1);
   p_feet_desired.resize(3,4);

   /* Error Calculations */
   error_x_lin.setZero(3,1); 
   error_dx_lin.setZero(3,1);
   error_R_lin.setZero(3,1);
   error_omega_lin.setZero(3,1);

   vbd_command_eigen.resize(3,1);
   vbd_command_eigen.setZero();

   /* Temporary, Internal Matrices */
   omegaHat.resize(3,3);   
   tempSkewMatrix3.resize(3,3);
   tempVector3.resize(3,1);

   tempSkewMatrix3.setZero();
   tempVector3.setZero();
   
   /* Robot Control variables for QP Matrices */
   A_control.resize(6,3*vblNUM_CONTACT_POINTS);
   b_control.resize(6,1);
   b_control_Opt.resize(6,1);
   f_unc.resize(12,1);
   C_control.resize(vblNUM_CONSTRAINTS_QP, 3*vblNUM_CONTACT_POINTS);
   C_times_f_opt.resize(vblNUM_CONSTRAINTS_QP, 1);
   C_control.setZero();

   /* Robot Control variables used in LQR for QP Optimization */
   A_LQR.resize(12,12);
   B_LQR.resize(12,3*vblNUM_CONTACT_POINTS);
   P_LQR.resize(12,12);
   f_ref_world.resize(3*vblNUM_CONTACT_POINTS,1);
   Q1_LQR.resize(12,12);
   Q2_LQR.resize(3*vblNUM_CONTACT_POINTS,3*vblNUM_CONTACT_POINTS);
   Q3_LQR.resize(3*vblNUM_CONTACT_POINTS,3*vblNUM_CONTACT_POINTS);
   s_LQR.resize(12,1);
   H_LQR.resize(2*vblNUM_VARIABLES_QP,2*vblNUM_VARIABLES_QP);

   /* Initialize Optimization variables */
   xOptPrev.setZero(12);
   yOptPrev.setZero(vblNUM_VARIABLES_QP + vblNUM_CONSTRAINTS_QP);
   for(int i = 0; i < vblNUM_VARIABLES_QP; i++)
   {
      xOpt_qpOASES[i] = 0.0;
      xOptPrev[i] = 0.0;
      xOpt_initialGuess[i] = 0.0;
   }

   for (int i = 0; i < 4; i++) {
    xOpt_initialGuess[3*i+2] = 20;
    xOptPrev[3*i+2] = 20;
   }


   for(int i = 0; i < vblNUM_VARIABLES_QP+vblNUM_CONSTRAINTS_QP; i++)
   {
      yOpt_qpOASES[i] = 0.0;
   }

   set_RobotLimits();
   set_worldData();

   cpu_time = 0.001;
   cpu_time_fixed = 0.001;
   qp_exit_flag = -1.0;

   qp_not_init = 1.0;
}


/* --------------- Primary Interface ------------------- */

void BalanceControllerVBL::updateProblemData(double* xfb_in,
                           double* p_feet_in,
                           double* p_feet_desired_in,
                           double* rpy_des_in,
                           double* rpy_act_in,
                           int stance_legs, 
                           bool firstRu)                          
{   

    for(int i=0;i<12;i++)
    {
    
        two_contact_stand_data.p_feet_in[i] = p_feet_in[i];
        two_contact_stand_data.p_feet_desired_in[i] = p_feet_desired_in[i];  
    }
    
   // Unpack inputs
   copy_Array_to_Eigen(quat_b_world, xfb_in, 4, 0);
   copy_Array_to_Eigen(x_COM_world, xfb_in, 3,4);
   copy_Array_to_Eigen(omega_b_world, xfb_in, 3,7);   
   copy_Array_to_Eigen(xdot_COM_world, xfb_in, 3,10);
   copy_Array_to_Eigen(p_feet, p_feet_in, 12,0);
   copy_Array_to_Eigen(p_feet_desired, p_feet_desired_in, 12,0);

   // Rotation matrix from quaternions
   quaternion_to_rotationMatrix(R_b_world, quat_b_world);

   // Express body angular velocity in body frame
   omega_b_body = R_b_world.transpose()*omega_b_world;
   omega_b_body_desired = R_b_world_desired.transpose()*omega_b_world_desired;

   
   // Update Centroidal Robot dynamics
   //update_A_control();
   
    rpy_des_real = rpy_des_in;
    calc_linear_error(rpy_des_real);
   
   
   
    update_A_LQR();
    update_B_LQR();
    
 
   // Solve Continuous-time Algebraic Ricatti Equation
   update_P_LQR();

   // Compute QP Problem data
   calc_H_qpOASES();
   calc_A_qpOASES();
   calc_g_qpOASES();
   
   int k = 0;
   for(int i=0; i<12;i++)
  {
      for (int j=0;j<12;j++)
      {
        two_contact_stand_data.P_LQR[k]= P_LQR(i,j);
        k +=1;
      }
  }

   update_log_variables(rpy_des_in, rpy_act_in);  

   cpu_time = cpu_time_fixed;
   nWSR_qpOASES = nWSR_fixed;  

}

void BalanceControllerVBL::SetContactData( double* contact_state_in,
                           double* min_forces_in,
                           double* max_forces_in)                                                  
{   

   // Unpack inputs
   copy_Array_to_Eigen(contact_state, contact_state_in,4,0);
   copy_Array_to_Eigen(minNormalForces_feet, min_forces_in,4,0);
   copy_Array_to_Eigen(maxNormalForces_feet, max_forces_in,4,0);
   
   calc_lb_ub_qpOASES();
   calc_lbA_ubA_qpOASES();
}


void BalanceControllerVBL::solveQP(double* xOpt)
{ 
   copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, 12);

   for(int i = 0; i < vblNUM_CONTACT_POINTS; i++)
   {
      tempVector3 = -R_b_world.transpose()*xOpt_eigen.segment(3*i,3);
      xOpt[3*i] = tempVector3(0);
      xOpt[3*i+1] = tempVector3(1);
      xOpt[3*i+2] = tempVector3(2);
   }
 
   calc_constraint_check();
   two_contact_stand_data_publish = two_contact_stand_data;
}


void BalanceControllerVBL::solveQP_nonThreaded(double* xOpt,double* dfVBL)
{
  // &cpu_time
  if(qp_not_init==1.0)
  {
      qp_exit_flag = QProblemObj_qpOASES.init(H_qpOASES,g_qpOASES,A_qpOASES,lb_qpOASES,ub_qpOASES,lbA_qpOASES,ubA_qpOASES,nWSR_qpOASES, &cpu_time, xOpt_initialGuess);     
      qp_not_init = 0.0;
      

      nWSR_initial = nWSR_qpOASES;
      cpu_time_initial = cpu_time;
  }else
  {
      qp_exit_flag = QProblemObj_qpOASES.init(H_qpOASES,g_qpOASES,A_qpOASES,lb_qpOASES,ub_qpOASES,lbA_qpOASES,ubA_qpOASES,nWSR_qpOASES,
       &cpu_time,xOpt_qpOASES,yOpt_qpOASES,&guessedBounds,&guessedConstraints);
  }
  
  QProblemObj_qpOASES.getPrimalSolution(xOpt_qpOASES);
  QProblemObj_qpOASES.getDualSolution( yOpt_qpOASES );

  QProblemObj_qpOASES.getBounds( guessedBounds );
  QProblemObj_qpOASES.getConstraints( guessedConstraints );

  two_contact_stand_data.exit_flag              = qp_exit_flag;
  two_contact_stand_data.nWSR                   = nWSR_qpOASES;
  two_contact_stand_data.cpu_time_microseconds  = cpu_time*1.0e6;
  copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, 12);

 
  // Combine reference control (f_des) and linear control (xOpt_eigen) inputs
  xOpt_combined = f_ref_world+xOpt_eigen;
  xOptPrev = xOpt_eigen;

  copy_Eigen_to_double(dfVBL,xOptPrev,12);
  
  // Centroid dynamics in world frame using solution to opitmization
  b_control_Opt = A_control*(xOpt_combined);

  // Transform forces into body coordinates
  for(int i = 0; i < vblNUM_CONTACT_POINTS; i++)
  {
    tempVector3 = -R_b_world.transpose()*xOpt_combined.segment(3*i,3);
    xOpt[3*i] = tempVector3(0);
    xOpt[3*i+1] = tempVector3(1);
    xOpt[3*i+2] = tempVector3(2);
  }

   
  
  QProblemObj_qpOASES.reset();

  calc_constraint_check();
  two_contact_stand_data_publish = two_contact_stand_data;
}

void BalanceControllerVBL::verifyModel(double* vbd_command)
{
   vbd_command_eigen = (1.0/mass)*A_control.block<3,12>(0,0)*xOpt_eigen-gravity;
   copy_Eigen_to_double(vbd_command, vbd_command_eigen, 3);
}

/* --------------- Control Math ------------ */

void BalanceControllerVBL::calc_linear_error(double* rpy_des_real)
{
  // Linear error for LQR
  error_x_lin = x_COM_world - x_COM_world_desired;
  error_dx_lin = xdot_COM_world - xdot_COM_world_desired;
  
  
  //per avere matrici A e B non mutevoli con la traiettoria desiderata
  rpyToR(R_b_world_desired_real, rpy_des_real);
  omega_b_body_desired_real = R_b_world_desired_real.transpose()*omega_b_world_desired;
  inverseCrossMatrix(0.5*(R_b_world_desired_real.transpose()*R_b_world-R_b_world.transpose()*R_b_world_desired_real),error_R_lin);
  error_omega_lin = omega_b_body - R_b_world.transpose()*R_b_world_desired_real*omega_b_body_desired_real;
 omega_b_body_desired_real = R_b_world_desired.transpose()*omega_b_world_desired;
  

  s_LQR << error_x_lin(0),error_x_lin(1),error_x_lin(2),error_dx_lin(0),error_dx_lin(1),error_dx_lin(2),error_R_lin(0),error_R_lin(1),error_R_lin(2),error_omega_lin(0),error_omega_lin(1),error_omega_lin(2);
  


  // Orientation error for data logging purposes
  matrixLogRot(R_b_world_desired*R_b_world.transpose(), orientation_error);

}

void BalanceControllerVBL::calc_constraint_check()
{
   C_times_f_opt = C_control*xOpt_eigen;

   for(int i = 0; i < vblNUM_VARIABLES_QP; i++)
   {
      two_contact_stand_data.f_opt[i] = xOpt_eigen(i);
      two_contact_stand_data.f_control[i] = xOpt_combined(i);
      two_contact_stand_data.f_unc[i] = f_unc[i];
   }

   for(int i = 0; i < 4; i++)
   {
      two_contact_stand_data.f_ref[i] = f_ref_world[3*i+2];
   }

   for(int i = 0; i < vblNUM_CONSTRAINTS_QP; i++)
   {
      two_contact_stand_data.lbA[i] = lbA_qpOASES[i];
      two_contact_stand_data.ubA[i] = ubA_qpOASES[i];
      two_contact_stand_data.C_times_f[i] = C_times_f_opt(i);
   }


}


/* --------------- QP Matrices and Problem Data ------------ */

void BalanceControllerVBL::update_A_control()
{

   // Update the A matrix in the controller notation A*f = b
   for(int i = 0; i < vblNUM_CONTACT_POINTS; i++)
   {
      tempSkewMatrix3.setIdentity();
      A_control.block<3,3>(0,3*i) << tempSkewMatrix3; // What is the purpose of this R_yaw

      tempVector3 << contact_state(i)*p_feet.col(i);
      crossMatrix(tempSkewMatrix3,tempVector3);
      A_control.block<3,3>(3,3*i) << tempSkewMatrix3;
   }   
}

void BalanceControllerVBL::update_A_LQR()
{
  // Temporary variables for block assignment
  Eigen::MatrixXd tempBlock; tempBlock.setZero(3,3);
  Eigen::VectorXd rd; rd.resize(3,1);
  Eigen::MatrixXd rd_hat; rd_hat.resize(3,3);

  // Update the A matrix in sdot = A*s+B*df
  tempSkewMatrix3.setIdentity();
  A_LQR.block<3,3>(0,3) << tempSkewMatrix3;
  A_LQR.block<3,3>(6,9) << tempSkewMatrix3;
  crossMatrix(tempSkewMatrix3, -omega_b_body_desired);
  A_LQR.block<3,3>(6,6) << tempSkewMatrix3;

  for(int i = 0; i<vblNUM_CONTACT_POINTS; i++)
  {
    tempVector3 << f_ref_world(3*i), f_ref_world(3*i+1), f_ref_world(3*i+2);
    crossMatrix(tempSkewMatrix3, tempVector3);
    tempBlock << tempBlock + Ig.inverse()*R_b_world_desired.transpose()*tempSkewMatrix3;
  }
  A_LQR.block<3,3>(9,0) << tempBlock;
  
  tempBlock.setZero();
  for(int i = 0; i<vblNUM_CONTACT_POINTS; i++)
  {
    tempVector3 << f_ref_world(3*i), f_ref_world(3*i+1), f_ref_world(3*i+2);
    rd << p_feet_desired.col(i);
    crossMatrix(rd_hat,rd);
    crossMatrix(tempSkewMatrix3,rd_hat*tempVector3);
    tempBlock << tempBlock + Ig.inverse()*R_b_world_desired.transpose()*tempSkewMatrix3;
  }
  A_LQR.block<3,3>(9,6) << tempBlock;

  
  /* NOTE: WE ASSUME THAT DESIRED ANGULAR VELOCITY IS ZERO
  if desired angular velocity of the body is nonzero, an additional block needs to be 
  added at A_LQR.block<3,3>(9,9) too account for Coriolis term */
}

void BalanceControllerVBL::update_B_LQR()
{
  Eigen::VectorXd rd;rd.resize(3,1);

  for(int i = 0; i<vblNUM_CONTACT_POINTS; i++)
  {
    tempSkewMatrix3.setIdentity();
    B_LQR.block<3,3>(3,i*3) << contact_state(i)*1/mass*tempSkewMatrix3;

    rd << p_feet_desired.col(i);
    crossMatrix(tempSkewMatrix3,rd);
    B_LQR.block<3,3>(9,3*i) << contact_state(i)*Ig.inverse()*R_b_world_desired.transpose()*tempSkewMatrix3;
  }
  

}

void BalanceControllerVBL::update_P_LQR()
{
  /* Solve the continuous time algebraic Ricatti equation via the Schur method */

  // Compute the Hamiltonian and its eigenvalues/vectors
  H_LQR.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(0,0) << A_LQR;
  H_LQR.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(0,vblNUM_VARIABLES_QP) << B_LQR*Q2_LQR.inverse()*B_LQR.transpose();
  H_LQR.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(vblNUM_VARIABLES_QP,0) << Q1_LQR;
  H_LQR.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP) << -A_LQR.transpose();
  EigenSolver<MatrixXd> es(H_LQR);

  // Create a 2nxn matrix U=[U11;U21] containing the eigenvectors of the stable eigenvalues
  Eigen::MatrixXcd U;U.setZero(2*vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP);
  Eigen::MatrixXcd U11;U11.setZero(vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP);
  Eigen::MatrixXcd U21;U21.setZero(vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP);
  Eigen::MatrixXcd P_complex;P_complex.setZero(vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP);
  std::complex<double> lambda;
  
  // U contains eigenvectors corresponding to stable eigenvalues of the Hamiltonian
  int j = 0;
  for(int i = 0;i < 2*vblNUM_VARIABLES_QP; i++)
  {
    lambda = es.eigenvalues()[i];
    if(lambda.real() < 0)
      {
        U.block<2*vblNUM_VARIABLES_QP,1>(0,j) << es.eigenvectors().col(i);
        j = j + 1;
      }
  }

  // Compute P based on U11*P = -U21;
  U11 = U.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(0,0);
  U21 = U.block<vblNUM_VARIABLES_QP,vblNUM_VARIABLES_QP>(vblNUM_VARIABLES_QP,0);
  P_complex = -U21*U11.inverse();
  P_LQR = P_complex.real();


  
  Eigen::VectorXd K_VBL = Q2_LQR.inverse()*B_LQR.transpose()*P_LQR;
  Eigen::Matrix<fpt,Dynamic,Dynamic>  tmp = P_LQR.cast <float>();
  Eigen::Matrix<fpt,Dynamic,Dynamic>  tmp1 = K_VBL.cast <float>(); 
    

  if(f_ref_world(5) >= 43.9961 && f_ref_world(8) >= 43.9961)
  { 
    
    Eigen::VectorXd K_VBL = Q2_LQR.inverse()*B_LQR.transpose()*P_LQR;
    Eigen::Matrix<fpt,Dynamic,Dynamic>  tmp = P_LQR.cast <float>();
    Eigen::Matrix<fpt,Dynamic,Dynamic>  tmp1 = K_VBL.cast <float>(); 
    

  }


  
  f_unc = -Q2_LQR.inverse()*B_LQR.transpose()*P_LQR*s_LQR;
  cost_to_go = s_LQR.transpose()*P_LQR*s_LQR;
  
  
  
}

void BalanceControllerVBL::calc_H_qpOASES()
{
  // Use LQR matrices to compute the QP cost matrix H
  //H_eigen = 2*(Q2_LQR);
  H_eigen = 2*(Q2_LQR+Q3_LQR);

  // Copy to real_t array (qpOASES data type)
   copy_Eigen_to_real_t(H_qpOASES, H_eigen, vblNUM_VARIABLES_QP, vblNUM_VARIABLES_QP);
}

void BalanceControllerVBL::calc_A_qpOASES()
{
   Eigen::Vector3d t1x; t1x << 1,0,0;
   Eigen::Vector3d t2y; t2y << 0,1,0;

   for(int i = 0; i<vblNUM_CONTACT_POINTS; i++)
   {
      C_control.block<1,3>(5*i+0,3*i) << -mu_friction*.7071*direction_normal_flatGround.transpose()
                                    + t1x.transpose();
      C_control.block<1,3>(5*i+1,3*i) << -mu_friction*.7071*direction_normal_flatGround.transpose()
                                    + t2y.transpose();
      C_control.block<1,3>(5*i+2,3*i) <<  mu_friction*.7071*direction_normal_flatGround.transpose()
                                    + t2y.transpose();                                    
      C_control.block<1,3>(5*i+3,3*i) <<  mu_friction*.7071*direction_normal_flatGround.transpose()
                                    + t1x.transpose();                                    
      C_control.block<1,3>(5*i+4,3*i) <<  direction_normal_flatGround.transpose();                                                                   
   }


   copy_Eigen_to_real_t(A_qpOASES, C_control, vblNUM_CONSTRAINTS_QP, vblNUM_VARIABLES_QP); 
}

void BalanceControllerVBL::calc_g_qpOASES()
{  

   
   g_eigen = 2*(B_LQR.transpose()*P_LQR.transpose()*s_LQR - Q3_LQR*xOptPrev);
   copy_Eigen_to_real_t(g_qpOASES, g_eigen, vblNUM_VARIABLES_QP, 1);
}


void BalanceControllerVBL::calc_lb_ub_qpOASES()
{
   for(int i = 0; i < vblNUM_CONTACT_POINTS; i++)
   {
      for(int j = 0; j < vblNUM_VARIABLES_PER_FOOT; j++)
     {
         lb_qpOASES[vblNUM_VARIABLES_PER_FOOT*i+j] = contact_state(i)*vblNEGATIVE_NUMBER;
         ub_qpOASES[vblNUM_VARIABLES_PER_FOOT*i+j] = contact_state(i)*vblPOSITIVE_NUMBER;
      }      
   }
   
}

void BalanceControllerVBL::calc_lbA_ubA_qpOASES()
{
   for(int i = 0; i < vblNUM_CONTACT_POINTS; i++)
   {
      lbA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i]   = contact_state(i)*vblNEGATIVE_NUMBER;      
      lbA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+1] = contact_state(i)*vblNEGATIVE_NUMBER;
      lbA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+2] = -mu_friction*f_ref_world(3*i+2)*.7071;
      lbA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+3] = -mu_friction*f_ref_world(3*i+2)*.7071;
      lbA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+4] = contact_state(i)*minNormalForces_feet(i)-f_ref_world(3*i+2);
      
      ubA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i]   = mu_friction*f_ref_world(3*i+2)*.7071;
      ubA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+1] = mu_friction*f_ref_world(3*i+2)*.7071;
      ubA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+2] = contact_state(i)*vblPOSITIVE_NUMBER;
      ubA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+3] = contact_state(i)*vblPOSITIVE_NUMBER;      
      ubA_qpOASES[vblNUM_CONSTRAINTS_PER_FOOT*i+4] = contact_state(i)*maxNormalForces_feet(i)-f_ref_world(3*i+2);
}
   
  
   
}


void BalanceControllerVBL::publish_data_lcm()
{  
   lcm->publish("CONTROLLER_two_contact_stand_data", &two_contact_stand_data_publish);
}

void BalanceControllerVBL::update_log_variables(double* rpy_des_in, double* rpy_act_in)
{
   for(int i = 0; i < 3; i++)
   {
      two_contact_stand_data.p_des[i] = x_COM_world_desired(i);
      two_contact_stand_data.p_act[i] = x_COM_world(i);
      two_contact_stand_data.rpy[i] = rpy_des_in[i];
      two_contact_stand_data.rpy_act[i] = rpy_act_in[i];
   }

   for(int i = 0; i < 12; i++)
   {
      two_contact_stand_data.s[i] = s_LQR(i);
   }

   two_contact_stand_data.cost_to_go = cost_to_go;



}
/* ------------ Set Parameter Values -------------- */

void BalanceControllerVBL::set_desiredTrajectoryData(double* rpy_des_in, double* p_des_in, double* omegab_des_in, double* v_des_in)
{
   x_COM_world_desired << p_des_in[0], p_des_in[1], p_des_in[2];
   rpyToR(R_b_world_desired, rpy_des_in);
   omega_b_world_desired << omegab_des_in[0], omegab_des_in[1], omegab_des_in[2];   
   xdot_COM_world_desired << v_des_in[0], v_des_in[1], v_des_in[2];
   
}

void BalanceControllerVBL::set_reference_GRF(double* f_ref_in)
{
  copy_Array_to_Eigen(f_ref_world,f_ref_in,12,0);
}

void BalanceControllerVBL::set_LQR_weights(double* x_weights_in, double* xdot_weights_in, double* R_weights_in, double* omega_weights_in, double alpha_control_in, double beta_control_in)
{
  Q1_LQR.setIdentity();
  Q2_LQR.setIdentity();
  Q3_LQR.setIdentity();

  for(int i = 0; i < 3; i++){
    Q1_LQR(i,i) = x_weights_in[i];
    Q1_LQR(i+3,i+3) = xdot_weights_in[i];
    Q1_LQR(i+6,i+6) = R_weights_in[i];
    Q1_LQR(i+9,i+9) = omega_weights_in[i];
  }

  for(int i = 0; i < 3*vblNUM_CONTACT_POINTS; i++) {
    Q2_LQR(i,i) = alpha_control_in;
    Q3_LQR(i,i) = beta_control_in;
  }

}

void BalanceControllerVBL::set_worldData()
{
   direction_normal_flatGround << 0,0,1;
   gravity << 0,0,9.81;
   direction_tangential_flatGround << 0.7071, 0.7071, 0;
   mu_friction = 0.05;
}

void BalanceControllerVBL::set_friction(double mu_in)
{
   mu_friction = mu_in;
}

void BalanceControllerVBL::set_mass(double mass_in)
{
   mass = mass_in;
}

void BalanceControllerVBL::set_inertia(double Ixx, double Iyy, double Izz)
{
   Ig << Ixx, 0, 0,
   0, Iyy, 0,
   0, 0, Izz;
}


void BalanceControllerVBL::set_RobotLimits()
{
   minNormalForces_feet << 10, 10, 10, 10;
   maxNormalForces_feet << 160, 160, 160, 160;
}

/* ------------ Utilities -------------- */


bool BalanceControllerVBL::getQPFinished()
{
   return QPFinished;
}

void BalanceControllerVBL::copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd &source, int nRows, int nCols )
{
   int count = 0;

   for(int i = 0; i < nRows; i++)
   {      
      for (int j = 0; j < nCols; j++)
      {
         target[count] = source(i,j);
         count++;
      }
   }  
}

void BalanceControllerVBL::copy_Eigen_to_double(double* target, Eigen::VectorXd &source, int length)
{
   for(int i = 0; i < length; i++)
   {
      target[i] = source(i);
   }
    
}

void BalanceControllerVBL::copy_Array_to_Eigen(Eigen::VectorXd &target, double* source, int len, int startIndex)
{
   for(int i = 0; i < len; i++)
   {
      target(i) = source[i+startIndex];
   }
}

void BalanceControllerVBL::copy_Array_to_Eigen(Eigen::MatrixXd &target, double* source, int len, int startIndex)
{
   for(int i = 0; i < len; i++)
   {
      target(i) = source[i+startIndex];
   }
}

void BalanceControllerVBL::copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t* source, int len)
{
   for(int i = 0; i < len; i++)
   {
      target(i) = source[i];
   }
}


void BalanceControllerVBL::matrixLogRot(const Eigen::MatrixXd & R, Eigen::VectorXd & omega) {
   double theta;
   double tmp = (R(0,0) + R(1,1) + R(2,2) - 1) / 2;
   if (tmp >=1.) {
      theta = 0;
   }
   else if (tmp <=-1.) {      
      theta = M_PI;
   }
   else {
      theta = acos(tmp);
   }

   omega << R(2,1)-R(1,2) , R(0,2)-R(2,0),R(1,0)-R(0,1);
   if (theta > 10e-5) {
      omega*=theta / (2*sin(theta));
   }
   else {
      omega/=2;
   }
}

void BalanceControllerVBL::crossMatrix(Eigen::MatrixXd &R, const Eigen::VectorXd &omega)
{
   R.setZero();
   R(0,1) = -omega(2);
   R(0,2) = omega(1);
   R(1,0) = omega(2);
   R(1,2) = -omega(0);
   R(2,0) = -omega(1);
   R(2,1) = omega(0);
}

void BalanceControllerVBL::inverseCrossMatrix(const Eigen::MatrixXd &R, Eigen::VectorXd &omega)
{
  omega(0) = R(2,1);
  omega(1) = R(0,2);
  omega(2) = R(1,0);
}

void BalanceControllerVBL::matrixExpOmegaCross(const Eigen::VectorXd & omega, Eigen::MatrixXd & R) {
   double theta = omega.norm();
   R.setIdentity();
   
   if (theta > 1e-9) {      
      omegaHat.setZero();      
      crossMatrix(omegaHat,omega/theta);      
      R += omegaHat*sin(theta) + omegaHat*omegaHat*(1-cos(theta));
   }
}

void BalanceControllerVBL::quaternion_to_rotationMatrix(Eigen::MatrixXd &R, Eigen::VectorXd &quat)
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

void BalanceControllerVBL::rpyToR(Eigen::MatrixXd &R, double* rpy_in)
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
