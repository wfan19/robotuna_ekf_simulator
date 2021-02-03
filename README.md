# RoboTuna EKF Simulator
A set of tools for Gazebo simulation and control of a quadcopter. Designed for unit testing a fast visual-inertial EKF for RoboTuna localization.

![](https://i.imgur.com/hrhXC06.png)

## Current features
- Quadcopter model with the following sensors:
    - Camera
    - IMU
    - Ground truth (Publishes to both odom topic and tf)
- Full attitude control
- Quadcopter ground truth pose visualization in RViz

## Usage
To start the simulation:\
`$ roslaunch robotuna_ekf_simulator Kwad_sim.launch`

To launch Rviz and propeller velocity control:\
`$ roslaunch robotuna_ekf_simulator Kwad_control.launch`

To run the attitude controller:\
`$ chmod +x scripts/controls/controller_node.py # Only needs to be done once`\
`# Note below that a node_name cli arg needs to be passed in`\
`$ rosrun robotuna_ekf_simulator controller_node.py controller_node`

You can then publish desired odometries to `/Kwad/cmd_odom`, and commanded throttles to `/Kwad/cmd_throttle`. Note: It usually takes a throttle value of 50 for the drone to start slowly ascending.

## TODO
**Control Features**
- Keyboard interface that inputs Euler angles
- PID velocity and position controller
- LQR/MPC trajectory follower? (Very very reach goal? See [this paper](<ttps://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis KTH - Francesco Sabatino.pdf>) and [this paper](https://arxiv.org/pdf/2006.05768.pdf)for state space models )

**World**
- Add Apriltags to world
- Create elevated platform for drone to start on

**Code**
- !!!! Organize the python scripts into a package !!!!
- Pull parameters from the ROS paremeter server (dynamic reconfigure), rather than being hardcoded
- Clean up controller_factory.py

## Acknowledgements
Quadcopter model and basic attitude control: A heavily modified version of [NisanthARao/ROS-Quadcopter-Simulation](https://github.com/NishanthARao/ROS-Quadcopter-Simulation)
