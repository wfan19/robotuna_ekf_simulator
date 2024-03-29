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
- A simulation world with various features and Apriltags (current pic needs updating)
- Euler angle based commands available through RQT

## Usage
To start the simulation:\
`$ roslaunch robotuna_ekf_simulator Kwad_sim.launch`

If this is your first time starting the simulation, (currently) you will need to go to the gazebo "Insert" tab on the left, and add the \<repo>/gazebo/models directory to the Gazebo model path. Otherwise, Gazebo will not know where to look for the Apriltag models, and the Apriltags will not display correctly.

To launch Rviz and propeller velocity control:\
`$ roslaunch robotuna_ekf_simulator Kwad_control.launch`

To run the attitude controller:\
`$ chmod +x scripts/controls/controller_node.py # Only needs to be done once`\
`$ rosrun robotuna_ekf_simulator controller_node.py controller_node`\
Note: running `controller_node.py` requires a node name argument. In this case, `controller_node` is used.

You can then publish desired Euler anglesto `/Kwad/cmd_euler`, and commanded throttles to `/Kwad/cmd_throttle`. \
Note: It usually takes a throttle value of 50 for the drone to overcome its own mass, and start slowly ascending.

## TODO
**Control Features**
- PID velocity and position controller
- LQR/MPC trajectory follower? (Very very reach goal? See [this paper](<https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis KTH - Francesco Sabatino.pdf>) and [this paper](https://arxiv.org/pdf/2006.05768.pdf) for state space models )

**Code**
- !!!! Organize the python scripts into a package !!!!
- Pull parameters from the ROS paremeter server (dynamic reconfigure), rather than being hardcoded
- Clean up controller_factory.py

## Acknowledgements
Quadcopter model and basic attitude control: A heavily modified version of [NisanthARao/ROS-Quadcopter-Simulation](https://github.com/NishanthARao/ROS-Quadcopter-Simulation)
