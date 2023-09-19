/*!
 * @file Simulation.h
 * @brief Main simulation class
 */

#ifndef PROJECT_SIMULATION_H
#define PROJECT_SIMULATION_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "ControlParameters/SimulatorParameters.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimUtilities/ImuSimulator.h"
#include "SimUtilities/SimulatorMessage.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"
#include "Utilities/Timer.h"

#include <mutex>
#include <queue>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include "simulator_lcmt.hpp"

#define SIM_LCM_NAME "simulator_state"

/*!
 * Top-level control of a simulation.
 * A simulation includes 1 robot and 1 controller
 * It does not include the graphics window: this must be set with the setWindow
 * method
 */
class Simulation {
  friend class SimControlPanel;
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Simulation(RobotType robot, Graphics3D* window,
                      SimulatorControlParameters& params, ControlParameters& userParams,
                      std::function<void(void)> ui_update);

  /*!
   * Explicitly set the state of the robot
   */
  void setRobotState(FBModelState<double>& state) {
    _simulator->setState(state);
  }

  void step(double dt, double dtLowLevelControl, double dtHighLevelControl);

  void addCollisionPlane(double mu, double resti, double height,
                         double sizeX = 20, double sizeY = 20,
                         double checkerX = 40, double checkerY = 40,
                         bool addToWindow = true);
  void addCollisionBox(double mu, double resti, double depth, double width,
                       double height, const Vec3<double>& pos,
                       const Mat3<double>& ori, bool addToWindow = true,
                       bool transparent = true);
  void addCollisionMesh(double mu, double resti, double grid_size,
                        const Vec3<double>& left_corner_loc,
                        const DMat<double>& height_map, bool addToWindow = true,
                        bool transparent = true);

  void lowLevelControl();
  void highLevelControl();

  /*!
   * Updates the graphics from the connected window
   */
  void updateGraphics();

  void runAtSpeed(std::function<void(std::string)> error_callback, bool graphics = true);
  void sendControlParameter(const std::string& name,
                            ControlParameterValue value,
                            ControlParameterValueKind kind,
                            bool isUser);

  void resetSimTime() {
    _currentSimTime = 0.;
    _timeOfNextLowLevelControl = 0.;
    _timeOfNextHighLevelControl = 0.;
  }

  ~Simulation() {
    delete _simulator;
    delete _robotDataSimulator;
    delete _imuSimulator;
    delete _lcm;
  }

  const FBModelState<double>& getRobotState() { return _simulator->getState(); }

  void stop() {
    _running = false;  // kill simulation loop
    _wantStop = true;  // if we're still trying to connect, this will kill us

    if (_connected) {
      _sharedMemory().simToRobot.mode = SimulatorMode::EXIT;
      _sharedMemory().simulatorIsDone();
    }
  }

  SimulatorControlParameters& getSimParams() { return _simParams; }

  RobotControlParameters& getRobotParams() { return _robotParams; }
  ControlParameters& getUserParams() { return _userParams; }

  bool isRobotConnected() { return _connected; }

  void firstRun();
  void buildLcmMessage();
  void loadTerrainFile(const std::string& terrainFileName,bool addGraphics = true);
  int kick(double dist_wx, double dist_wy,double dist_wz);

 private:
  void handleControlError();
  Graphics3D* _window = nullptr;

  std::mutex _robotMutex;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  ImuSimulator<double>* _imuSimulator = nullptr;
  SimulatorControlParameters& _simParams;
  ControlParameters& _userParams;
  RobotControlParameters _robotParams;

  size_t _simRobotID, _controllerRobotID;
  Quadruped<double> _quadruped;
  FBModelState<double> _robotControllerState;
  FloatingBaseModel<double> _model;
  FloatingBaseModel<double> _robotDataModel;
  DVec<double> _tau;
  DynamicsSimulator<double>* _simulator = nullptr;
  DynamicsSimulator<double>* _robotDataSimulator = nullptr;
  std::vector<ActuatorModel<double>> _actuatorModels;
  SpiCommand _spiCommand;
  SpiData _spiData;
  SpineBoard _spineBoards[4];
  TI_BoardControl _tiBoards[4];
  RobotType _robot;
  lcm::LCM* _lcm = nullptr;

  std::function<void(void)> _uiUpdate;
  std::function<void(std::string)> _errorCallback;
  bool _running = false;
  bool _connected = false;
  bool _wantStop = false;
  double _desiredSimSpeed = 1.;
  double _currentSimTime = 0.;
  double _currentRealTime = 0.;
  double _timeOfNextLowLevelControl = 0.;
  double _timeOfNextHighLevelControl = 0.;
  s64 _highLevelIterations = 0;
  simulator_lcmt _simLCM;

  
  int i = 1 ;
  int end_kick = 0;
  int j = 0;
  double dist_wx;
  double dist_wy;
  double dist_wz;
  bool firstTime = true;

  
  /*----------Kick ROA NN disturbances wx-wy inside ------------*/

  
  //double wx[10] = {-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0,-5.0};
  //double wy[10] = {3.0,2.5,2.0,1.5,1.0,0.0,-0.5,-1.0}; 

  //double wx[10] = {-4.5,-4.5,-4.5,-4.5,-4.5,-4.5,-4.5,-4.5,-4.5,-4.5};
  //double wy[10] = {3.0,2.5,2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5};
 
   //double wx[13] = {-4.0,-4.0,-4.0,-4.0,-4.0,-4.0,-4.0,-4.0,-4.0,-4.0,-3.5,-3.5,-3.5};
   //double wy[13] = { 3.0, 2.5, 2.0, 1.5, 1.0, 0.5,0.0, -0.5,-1.0,-1.5,3.0,2.5,-2.0};

  //double wx[10] = {-3.5,-3.5,-3.5,-3.5,-3.5,-3.5,-3.5};
  //double wy[10] = {2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5};
   
  //double wx[11] = {-3.0,-3.0,-3.0,-3.0,-3.0,-3.0,-3.0,-3.0,-3.0,-2.5};
  //double wy[11] = { 3.0, 2.5, 2.0,1.0, 0.5, 0.0,-0.5,-1.0,-1.5,2.5};

   //double wx[11] = {-2.5,-2.5,-2.5,-2.5,-2.5,-2.5,-2.5,-2.5,-2.5,-2.0};
   //double wy[11] = {2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.0}; 

   //double wx[11] = {-2.0,-2.0,-2.0,-2.0,-2.0,-2.0,-2.0,-2.0,-2.0,-2.0};
  //double wy[11] = {2.5,-2.5,2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5};
 
   //double wx[10] = {-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5};
   //double wy[10] = {2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5};

  //double wx[11] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-0.5};
  //double wy[11] = {2.0,1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,2.0};

  //double wx[10] = {-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,0.0};
  //double wy[10] = {1.5,1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,1.5};

  //double wx[11] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.5};
  //double wy[11] = {1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,1.5,1.0};

  //double wx[11] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1.0,1.0,1.0};
  //double wy[11] = {0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,1.0,0.5,0.0};

  //double wx[7] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  //double wy[7] = {-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,1.0};

  //double wx[5] = {1.0,1.5,1.5,1.5,1.5};
  //double wy[5] = {-3.5,1.0,0.5,0.0,-0.5};

  //double wx[5] = {1.5,1.5,1.5,1.5,1.5};     
  //double wy[5] = {-1.0,-1.5,-2.0,-2.5,-3.0};

  //double wx[7]   {1.5,2.0,2.0,2.0,2.0,2.0,2.0};
  //double wy[7]= {-3.5,1.0,0.5,0.0,-0.5,-1.0,-1.5};

  //double wx[11] = {2.0,2.0,2.0,2.0,2.5,2.5,2.5,2.5,2.5,2.5,2.5};
  //double wy[11] = {-2.0,-2.5,-3.0,-3.5,1.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0};
  
  //double wx[11] = {2.5,2.5,2.5,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0};
  //double wy[11] = {-2.5,-3.0,-3.5,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0}; 
 
  //double wx[12] = {3.0,3.0,3.0,3.0,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5};
  //double wy[12] = {-3.5,-4.0,-4.5,-5.0,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5};

  //double wx[13] = {3.5,3.5, 3.5, 4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0,4.0}; 
  //double wy[13] = {-4.0,-4.5,-5.0,0.5,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5,-4.0}; 

  //double wx[11] = {4.0,4.0,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5};
  //double wy[11] = {-4.5,-5.0,0.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5,-4.0};

  double wx[12] = {4.5,4.5,5.0,5.0,5.0,5.0,5.0, 5.0,5.0,5.0,5.0,5.0};
  double wy[12] = {-4.5,-5.0,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5,-4.0,-4.5,-5.0};



};

#endif  // PROJECT_SIMULATION_H
