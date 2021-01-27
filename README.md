# RoboTuna EKF Simulator
A set of tools for Gazebo simulation and control of a quadcopter. Designed for unit testing a fast visual-inertial EKF for RoboTuna localization.

## Current features
- Spawns quadcopter with the following sensors:
    - Camera
    - IMU
    - Ground truth (Publishes to both odom topic and tf)
- Basic attitude stabilization

## Usage
To start the simulation:\
`$ roslaunch robotuna_ekf_simulator Kwad_sim.launch`

To launch Rviz and propeller velocity control:\
`$ roslaunch robotuna_ekf_simulator Kwad_control.launch`

To run the basic attitude controller:\
`$ chmod +x scripts/control.py # Only needs to be done once`\
`$ rosrun robotuna_ekf_simulator control.py`

## TODO
**Control**
- DIY PID attitude controller
- Cascading PID velocity/position controller (Reach goal?)
- LQR velocity/position controller? (Very very reach goal? See [this paper](<ttps://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis KTH - Francesco Sabatino.pdf>) for state space model)

**World**
- Add Apriltags to world

## Acknowledgements
Quadcopter model and basic attitude control: A heavily modified version of [NisanthARao/ROS-Quadcopter-Simulation](https://github.com/NishanthARao/ROS-Quadcopter-Simulation)
