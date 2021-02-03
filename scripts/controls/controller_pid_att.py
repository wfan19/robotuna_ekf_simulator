#!/usr/bin/env python3

from pidff import *
from controller_base import Controller
from controller_pid_bodyrate import ControllerPIDBodyrate
from utils import *

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64MultiArray

class AttCtrlParams:
    kP = 1
    yaw_weight = 0.5
    def __init__(self, kP = 1, yaw_weight = 0.5):
        self.kP = kP
        self.yaw_weight = yaw_weight

class ControllerPIDAtt(Controller):
    def __init__(self, time: float, attitude_params: AttCtrlParams, bodyrate_params: PIDFFParams):
        super().__init__(time)
        self.controller_bodyrate = ControllerPIDBodyrate(time, bodyrate_params)
        self.attitude_params = attitude_params

    # TODO: Create overloaded version that takes in vectors rather than Odoms
    def control(self, time: float, thrust_cmd: float, reference_odom: Odometry, current_state: Odometry, v_sp_attitude: np.array=None, v_cur_attitude: np.array=None):

        # Convert target pose into np vector
        v_ref_pos, v_ref_attitude = ros_pose_to_np_arrays(reference_odom.pose.pose)            # Target position and heading

        # Convert current states into np vector
        v_cur_pos, v_cur_attitude = ros_pose_to_np_arrays(current_state.pose.pose)   # Current position and heading
        v_cur_vel_lin = ros_vector3_to_np_array(current_state.twist.twist.linear)   # Current linear velocity
        v_cur_vel_ang = ros_vector3_to_np_array(current_state.twist.twist.angular)  # Current angular velocity

        # Check if quaternion is invalid
        if np.linalg.norm(v_ref_attitude) == 0 or np.linalg.norm(v_cur_attitude) == 0:
            # TODO: Maybe make this return the most recent previous command?
            print("[ControllerPIDAtt]: Invalid (zero) quaternion received - returning all zero commands")
            out_msg = Float64MultiArray()
            out_msg.data = [0, 0, 0, 0]
            return out_msg

        # Convert quaternion vectors to SciPy rotations
        rot_ref_attitude = Rotation.from_quat(v_ref_attitude)
        rot_cur_attitude = Rotation.from_quat(v_cur_attitude)

        # Quaternion based attitude controller
        # Generates a bodyrate setpoint
        v_sp_vel_ang = self.control_attitude(rot_ref_attitude, rot_cur_attitude)

        # Control body rate with the PID based bodyrate controller
        return self.controller_bodyrate.control(time, thrust_cmd, v_sp_vel_ang=v_sp_vel_ang, v_cur_vel_ang=v_cur_vel_ang)

    # Note: The desired orientation we are passed should already be from Full Attitude Control
    # Thus the function parameter is named "rot_sp_full" for being the Full Attitude Control rotation setpoint
    def control_attitude(self, rot_sp_full: Rotation, rot_cur_attitude: Rotation):
        # An implementation of the quaternion-based attitude control described in this paper:
        # https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
        
        # PX4 implementation can be found in update() here:
        # https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp 

        # Notes on terminology:
        # For target orientation quaternion:
            # Paper     "q_cmd"
            # Px4 code  "q_d"
            # Here      "q_sp" or "rot_sp"

        # For frame-of-reference basis vectors:
            # Paper     "e^B"
            # Px4 code  "e"
            # Here      "body"

        # For distinguishing between Reduced and Full:
            # Paper     "red"
            # Px4 code  "red"
            # Here      "reduced"
        
        ##############################
        # Generate quaternion setpoint
        ##############################
        ## Generate quaternion setpoint via Reduced (yaw-uncontrolled) Attitude Control:
        # See section 3.2.1 in the paper
        
        # Get current bodyZ:
        # The third column of rotation matrices encode the corresponding desired position of the Z basis vector
        body_z_cur = rot_cur_attitude.as_matrix()[:, 2]
        body_z_sp = rot_sp_full.as_matrix()[:, 2]
        
        # Calculate reduced rotation setpoint
        q_sp_reduced = self.quat_between_vectors(body_z_cur, body_z_sp)
        rot_sp_reduced = Rotation.from_quat(q_sp_reduced)
        if np.abs(q_sp_reduced)[0] == 1 or np.abs(q_sp_reduced)[1] == 1:
            # Corner case: vehicle and thrust have opposite directions
            # Here we just ignore the reduced control
            rot_sp_reduced = rot_sp_full
        else:
            # Transform rot_sp_reduced from body to world frame
            rot_sp_reduced = rot_sp_reduced * rot_cur_attitude
        # Finished calculating reduced rotation setpoint

        ## Mixing reduced and full attitude control to get final quaternion setpoint
        # See section 3.2.3 and eq(53) in the paper
        rot_reduced_to_full = rot_sp_reduced.inv() * rot_sp_full
        rot_reduced_to_full = self.canonicalize_rotation(rot_reduced_to_full)
        q_reduced_to_full = rot_reduced_to_full.as_quat()

        # Constrain domains for arccos and arcsin
        q_reduced_to_full[2] = np.clip(q_reduced_to_full[2], -1, 1)
        q_reduced_to_full[3] = np.clip(q_reduced_to_full[3], -1, 1)

        yaw_weight = self.attitude_params.yaw_weight
        
        # Mixing quaternion. See eq(53)
        q_mixer = np.array([
            0,
            0,
            np.sin(yaw_weight * np.arcsin(q_reduced_to_full[2])),
            np.cos(yaw_weight * np.arccos(q_reduced_to_full[3]))
        ])
        
        # Apply reduced-to-full rotation to the mixing orientation
        rot_mixer = Rotation.from_quat(q_mixer)
        rot_sp_mixed = rot_sp_reduced * rot_mixer
        # Finished mixing reduced and full rotation setpoints

        ##########################################
        # Apply quaternion setpoint to Control Law
        ##########################################
        ## Convert quaternion setpoint to angular rates via the control law described by eq(23) in the paper
        
        # Get attitude error (as rotation) between current and mixed setpoint
        rot_error = rot_cur_attitude.inv() * rot_sp_mixed

        # Canonicalize and get error quaternion
        rot_error = self.canonicalize_rotation(rot_error)
        q_error = rot_error.as_quat()

        # Apply control law
        bodyrates_sp = 2 * self.attitude_params.kP * q_error[0:3]
        bodyrates_sp = np.reshape(bodyrates_sp, (3, 1))

        return bodyrates_sp
    
    # Maybe this shouldn't be a class member function?
    def canonicalize_rotation(self, rotation: Rotation):
        q_rotation = rotation.as_quat()
        for i in q_rotation:
            if i < 0:
                return Rotation.from_quat(-1 * q_rotation)
        return rotation
        
    def quat_between_vectors(self, src: np.array, dest: np.array):
        # An implementation of the Quaternion constructor given two vectors from here:
        # https://github.com/PX4/PX4-Matrix/blob/master/matrix/Quaternion.hpp
        
        # Also see this thread on Stackoverflow for a derivation with clean algebra:
        # https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        
        q_out = np.array([0. , 0., 0., 1.])
        dot = np.dot(src, dest)
        cross = np.cross(src, dest)
        
        # Handle the corner case where the two vectors are co-linear/parallel and opposite:
        if (np.linalg.norm(cross) == 0 and dot < 0):
            cross = np.abs(src)
            if cross[0] < cross[1]:
                if cross[0] < cross[2]:
                    cross = np.array([1, 0, 0])
                else:
                    cross = np.array([0, 0, 1])
            else:
                if cross[1] < cross[2]:
                    cross = np.array([0, 1, 0])
                else:
                    cross = np.array([0, 0, 1])
            q_out[3] = 0
            cross = np.cross(src, cross)
        else:
            q_out[3] = dot + np.linalg.norm(src) * np.linalg.norm(dest)

        q_out[0:3] = cross
        
        q_out = q_out / np.linalg.norm(q_out)
        return q_out
