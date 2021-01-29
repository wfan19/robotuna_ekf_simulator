import unittest

from pidff import *

class TestPIDFF(unittest.TestCase):
    def test_scalar(self):
        t0 = 10000
        t_current = 10100
        ref = 1
        meas = 0.5

        gains = PIDFFGains(kP = 100, kI = 1, kD = 10, kFF = 5, i_min = 10, i_max = 10)
        pidff = PIDFF(gains, t0)

        control_val = pidff.update(t_current, ref, meas)
        self.assertEqual(control_val, 65.05)

    def test_np_array(self):
        import numpy as np

        t0 = 10000
        t_current = 10100

        v_ref = np.array([1, 0, -1]).T
        v_meas = np.array([0.5, 0, 100]).T

        gains = PIDFFGains(kP = 100, kI = 1, kD = 10, kFF = 5, i_min = 10, i_max = 10)
        pidff = PIDFF(gains, t0)
        v_pidff = PIDFF(gains, t0)

        v_control_val = v_pidff.update(t_current, v_ref, v_meas)

        control_val_1 = pidff.update(t_current, v_ref[0], v_meas[0])
        
        pidff.reset(t0)
        control_val_2 = pidff.update(t_current, v_ref[1], v_meas[1])
        
        pidff.reset(t0)
        control_val_3 = pidff.update(t_current, v_ref[2], v_meas[2])

        print(f"\nScalar test vals: [{control_val_1}, {control_val_2}, {control_val_3}]")
        print(f"Np array test vals: {v_control_val}")
        # Check that the vector version is the same as the scalar version
        self.assertEqual(control_val_1, v_control_val[0])
        self.assertEqual(control_val_2, v_control_val[1])
        self.assertEqual(control_val_3, v_control_val[2])

        # TODO: A test for multiple controller updates over time? That way we can test for the I and D terms

if __name__ == '__main__':
    unittest.main()
