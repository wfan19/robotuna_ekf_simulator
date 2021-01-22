# RoboTuna EKF Simulator
A set of tools for Gazebo simulation and control of a quadcopter.

## Current features
- Spawns quadcopter with the following sensors:
    - Camera
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
**Model**
- Modify drone model to un-obstruct camera view

**Control**
- PID attitude controller
- Cascading PID velocity controller (Reach goal?)
- LQR velocity/position controller? (Very very reach goal? See Drake for example impl)

## Acknowledgements
Quadcopter model and basic attitude control: [NisanthARao/ROS-Quadcopter-Simulation](https://github.com/NishanthARao/ROS-Quadcopter-Simulation)
