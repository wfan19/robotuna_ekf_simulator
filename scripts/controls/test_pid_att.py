#!/usr/bin/env python3

from pidff import PIDFFParams
from controller_pid_att import AttCtrlParams
from controller_factory import *

from nav_msgs.msg import Odometry

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

def scipy_rot_to_ros_odom(rotation: Rotation):
    odom_out = Odometry()
    q_rotation = rotation.as_quat()

    odom_out.pose.pose.orientation.x = q_rotation[0]
    odom_out.pose.pose.orientation.y = q_rotation[1]
    odom_out.pose.pose.orientation.z = q_rotation[2]
    odom_out.pose.pose.orientation.w = q_rotation[3]

    return odom_out

# Ref and meas are in DEGREES
# Euler angles for easier testing
# Order: xyz (roll pitch yaw)
def standard_test(ref: list, meas: list):
        # Testing body rate control of pitch at constant throttle
        t0 = 1000
        t_control = 1010
        
        thrust = 10
        bodyrate_kP = np.diag([0.5, 0.5, 0.5])

        bodyrate_params = PIDFFParams(kP=bodyrate_kP)
        attitude_params = AttCtrlParams(kP = 1, yaw_weight = 0.5)

        mControllerFactory = ControllerFactory(bodyrate_params=bodyrate_params, attitude_params=attitude_params)
        controller = mControllerFactory.create_controller("Attitude", t0)

        rot_target = Rotation.from_euler("xyz", ref, degrees=True)
        rot_current = Rotation.from_euler("xyz", meas, degrees=True) # Current state: Left side is currently up
        
        odom_ref = scipy_rot_to_ros_odom(rot_target)
        odom_meas = scipy_rot_to_ros_odom(rot_current)

        control_vals = controller.control(time=t_control, thrust_cmd=thrust, reference_odom=odom_ref, current_state=odom_meas)
        return control_vals.data

class TestPIDAttitude(unittest.TestCase):
    def test_attitude_roll_only(self):
        # Testing body rate control of pitch at constant throttle

        # Currently flat
        # Aiming to get left side up
        # Therefore we should see left > right
        v_control_vals = standard_test(ref=[45, 0, 0], meas=[0, 0, 0])
        print(f"\nroll-only Control values: {np.abs(v_control_vals)}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_left = np.mean(v_control_vals[1:3])
        mean_right = np.mean([v_control_vals[0], v_control_vals[3]])

        # X axis from the front = positive roll is left-up
        self.assertTrue(mean_left > mean_right)

    def test_attitude_pitch_only(self):

        # Currently flat
        # Aiming to get nose down
        # Therefore we should see front < back
        v_control_vals = standard_test(ref=[0, 45, 0], meas=[0, 0, 0])
        print(f"\nPitch-only Control values: {np.abs(v_control_vals)}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_front = np.mean(v_control_vals[0:2])
        mean_back = np.mean(v_control_vals[2:4])

        # Y axis from the left = positive pitch is nose-down
        self.assertTrue(mean_front > mean_back)

    def test_attitude_yaw_only(self):

        # Currently flat
        # Aiming to get nose left
        # Therefore we should see fl_br > fr_bl
        v_control_vals = standard_test(ref=[0, 0, 45], meas=[0, 0, 0])
        print(f"\nYaw-only Control values: {np.abs(v_control_vals)}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_fl_br = np.mean([v_control_vals[1], v_control_vals[3]])
        mean_fr_bl = np.mean([v_control_vals[0], v_control_vals[2]])

        # Z axis from the top = positive yaw is nose left
        self.assertTrue(mean_fl_br > mean_fr_bl)

if __name__ == '__main__':
    unittest.main()
