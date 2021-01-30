#!/usr/bin/env python3

from pidff import PIDFFParams
from controller_factory import *

import unittest

import numpy as np

def standard_test(ref, meas, kP = np.diag([0.1, 0.1, 0.1])):
        # Testing body rate control of pitch at constant throttle
        t0 = 1000
        t_control = 1010
        
        thrust = 10

        bodyrate_params = PIDFFParams(kP=kP)
        mControllerFactory = ControllerFactory(bodyrate_params = bodyrate_params)
        controller = mControllerFactory.create_controller("Bodyrate", t0)

        control_vals = controller.control(t_control, v_sp_vel_ang=ref, v_cur_vel_ang=meas, thrust_cmd=thrust)
        return control_vals

class TestPIDBodyrate(unittest.TestCase):
    def test_bodyrate_pitch_only(self):
        # Testing body rate control of pitch at constant throttle
        ref = np.array([[0, 1, 0]]).T
        meas = np.array([[0, 0, 0]]).T

        control_vals = standard_test(ref, meas)
        v_control_vals = np.array(control_vals.data)
        print(f"\nPitch-only Control values: {control_vals.data}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_front = np.mean(v_control_vals[0:2])
        mean_back = np.mean(v_control_vals[2:4])

        # Y axis from the left = positive pitch is nose-down
        self.assertTrue(mean_back > mean_front)

    def test_bodyrate_roll_only(self):
        # Testing body rate control of roll at constant throttle
        ref = np.array([[1, 0, 0]]).T
        meas = np.array([[0, 0, 0]]).T
        control_vals = standard_test(ref, meas)
        v_control_vals = np.array(control_vals.data)
        print(f"\nRoll-only Control values: {control_vals.data}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_left = np.mean(v_control_vals[1:3])
        mean_right = np.mean([v_control_vals[0], v_control_vals[3]])

        # X axis from the front = positive roll is left-up
        self.assertTrue(mean_left > mean_right)

    def test_bodyrate_yaw_only(self):
        # Testing body rate control of roll at constant throttle
        ref = np.array([[0, 0, 1]]).T
        meas = np.array([[0, 0, 0]]).T
        control_vals = standard_test(ref, meas)
        v_control_vals = np.array(control_vals.data)
        print(f"\nYaw-only Control values: {control_vals.data}")
        
        v_control_vals = np.abs(v_control_vals.data)
        mean_fl_br = np.mean([v_control_vals[1], v_control_vals[3]])
        mean_fr_bl = np.mean([v_control_vals[0], v_control_vals[2]])

        # Z axis from the top = positive yaw is nose left
        self.assertTrue(mean_fl_br > mean_fr_bl)

if __name__ == '__main__':
    unittest.main()
