#!/usr/bin/env python3

from pidff import *
from controller_base import Controller
from utils import *

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64MultiArray

class ControllerPIDBodyrate(Controller):
    def __init__(self, time: float, body_rate_params: PIDFFParams):
        super().__init__(time)
        self.controller_body_rate = PIDFF(body_rate_params, time=time)
        
        # Mixing matrix
        # Configures how the linear combination of desired body rates + thrust determine the propeller speeds
        # Possible TODO: Figure out a way to pass the matrix in from rosparam without using rospy within here?
        self.mixing_matrix = np.array([
            # Columns: 
            # roll, pitch, yaw, thrust
            [-0.71, -0.71, -1, 1],    # Front right
            [0.71, -0.71, 1, 1],    # Front left
            [0.71, 0.71, -1, 1],    # Back left
            [-0.71, 0.71, 1, 1]   # Back right
        ])

        # Sign matrix
        # Two of the propellers need to spin in reverse to the other two
        self.sign_matrix = np.diag([1, -1, 1, -1])
    
    # TODO: Create overloaded version that takes vectors rather than structs
    def control(self,
            time: float,
            thrust_cmd: float,
            reference_odom: Odometry=None,
            current_state: Odometry=None,
            v_sp_vel_ang: np.array=None,
            v_cur_vel_ang: np.array=None
        ):
        # Convert current and target angular velocities into np vector
        # ROS twist is XYZ (roll pitch yaw based on body frame orientation)
        if v_sp_vel_ang is None or v_sp_vel_ang.size != 3:
            if reference_odom is not None:
                v_sp_vel_ang = ros_vector3_to_np_array(reference_odom.twist.twist.angular)
            else:
                # Throw error
                quit()

        if v_cur_vel_ang is None or v_cur_vel_ang.size != 3:
            if current_state is not None:
                v_cur_vel_ang = ros_vector3_to_np_array(current_state.twist.twist.angular)
            else:
                # Throw error
                quit()

        # Control body rate with PID
        v_angular_accel_sp = self.controller_body_rate.update(time, v_sp_vel_ang, v_cur_vel_ang)

        # v_inputs: vector of [roll_accel; pitch_accel; yaw_accel; thrust_cmd]
        v_inputs = np.concatenate((v_angular_accel_sp, [[thrust_cmd]]), 0)

        # Mixing
        # Deriving propeller outputs as a linear combination of desired angular rates
        v_outputs = self.sign_matrix @ self.mixing_matrix @ v_inputs
        
        out_msg = Float64MultiArray()
        out_msg.data = v_outputs[:, 0]

        print("==================================")
        print(f"Bodyrate ref: {np.reshape(v_sp_vel_ang, (3,))}")
        print(f"meas: {np.reshape(v_cur_vel_ang, (3,))}")
        print(f"error: {np.reshape(v_sp_vel_ang - v_cur_vel_ang, (3,))}")
        print(f"Roll, pitch, yaw, thrust: {np.reshape(v_inputs, (4, ))}")
        print(f"Control out: {out_msg.data}")

        return out_msg