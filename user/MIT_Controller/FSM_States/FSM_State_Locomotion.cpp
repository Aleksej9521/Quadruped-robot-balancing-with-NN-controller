/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  if(_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        27 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else if(_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        33 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else{
    assert(false);
  }


  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cMPCOld->initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}

extern rc_control_settings rc_control;

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  if(locomotionSafe()) {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_LOCOMOTION:
        break;

//       case K_BALANCE_STAND:
//         // Requested change to BALANCE_STAND
//         this->nextStateName = FSM_StateName::BALANCE_STAND;
// 
//         // Transition time is immediate
//         this->transitionDuration = 0.0;
// 
//         break;

      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        this->transitionDuration = 0.;
        break;

      case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        break;


      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_LOCOMOTION << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
  } else {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
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
TransitionData<T> FSM_State_Locomotion<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
//     case FSM_StateName::BALANCE_STAND:
//       LocomotionControlStep();
// 
//       iter++;
//       if (iter >= this->transitionDuration * 1000) {
//         this->transitionData.done = true;
//       } else {
//         this->transitionData.done = false;
//       }
// 
//       break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
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

template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.18)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 9.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
 

  cMPCOld->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for(int leg(0); leg<4; ++leg){
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

 
  for(int leg(0); leg<4; ++leg){
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }

}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg) {
  // Impedance control for the stance leg
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
