# Controls
This directory currently houses all of the drone control related Python script files. The files can be divided into three types: General files, Controller files, and Unit Tests.

## General
`controller_node`: The ROS node executable. Creates a `Controller` object via a `ControllerFactory` instance. All actual control logic is abstracted away

`controller_factory`: A factory object that manages the creation logic for all controllers (currently only two). Give it all of the controller parameters, and it will give you any controller you want.

`pidff`: A basic PID + Feedforward library. Give it measurement, reference, and time updates, and it will give you commands. All `pid` controllers, other than `controller_pid_att` are based on this.

`utils`: A couple of general utilty functions, currently just for converting data types

## Controllers

`controller_base`: The base controller interface class. The only method is `control()`, which takes reference and current states, current time, and a throttle value, and outputs propeller speeds. Actual implementation of `control()` is left to the children controllers.

`controller_pid_bodyrate`: A basic PID based bodyrate (angular velocity) controller. Also contains "mixing" - combining the separate roll pitch yaw accelerations into final propeller commands.

`controller_pid_att`: An implementation of the Pixhawk 4 quaternion based proportional attitude controller, as described and analyzed in [this paper](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf). The Pixhawk implementation can be found [here](https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp). Wraps a `controller_pid_bodyrate` bodyrate controller.

## Unit Tests

`test_pidff`: A set of unit tests for validating that `pidff` works for both scalar/vector values, and both scalar/matrix gains.

`test_pid_bodyrate`: A set of roll, pitch, yaw rate tests to make sure that `controller_pid_bodyrate` responds to bodyrate commands correctly.

`test_pid_attitude`: A set of roll, pitch, yaw tests that make sure that `controller_pid_att` responds to attitude commands correctly. A test using real Gazebo simulation data is also incorporated.