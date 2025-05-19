## Two leg balancing NN
This repository is related to the article "A stable and safe method for two-leg balancing of a quadruped robot using a neural-network-based controller" Li Noce A., Patanè L., Arena P., https://doi.org/10.1016/j.robot.2024.104901 . For testing the neural network set use_twolegs equal to 1 before starting. At the beginning, the robot is in recovery mode state. To switch on two legs balancing set control_mode equal to 2. 

It is possible to run the following examples:
- Step variation along roll,pitch and yaw: change step_roll, step_pitch e step_yaw equal to 1 respectively. 
- Analysis of RoA: it is possible to evaluate the experimental ROA using the automatic kick. To do that, set roa_test to 1 before set control_mode.

If you have some internet problems, you can do:
 
cd /yourpath/scripts
 ./No_internet_lcm.sh (set this file as executable)


This simulation environment is based on the MIT Mini Cheetah code (https://github.com/mit-biomimetics/Cheetah-Software).

Please note to change the pathto the weights with your own path. 

## Cheetah-Software
This repository contains the Robot and Simulation software project. 

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain third party libraries.

## Build
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

If you change LCM types, you'll need to run:

 cd /yourpath/scripts
 ./make_types.sh (set this file as executable)


```
## Run simulator
To run the simulator:
1. Open the control board
```
sudo ./sim/sim
```
2. In the another command window, run the robot control code
``'
sudo ./user/MIT_Controller/mit_ctrl m s
```
m: Mini Cheetah s: simulation



## Dependencies:

- sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
- Qt 5.14 - [https://www.qt.io/download-qt-installer](https://download.qt.io/archive/qt/5.14/5.14.2/)
- LCM 1.3.1 - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

