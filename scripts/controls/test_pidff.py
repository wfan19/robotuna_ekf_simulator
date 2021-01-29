import unittest

from pidff import *

class TestPIDFF(unittest.TestCase):
    def test_scalar(self):
        t0 = 10000
        t_current = 10100
        ref = 1
        meas = 0.5

        gains = PIDFFGains(kP = 100, kI = 1, kD = 10, kFF = 5, i_min = -10, i_max = 10)
        pidff = PIDFF(gains, t0)

        control_val = pidff.update(t_current, ref, meas)
        self.assertEqual(control_val, 65.05)

    def test_np_array(self):
        import numpy as np

        t0 = 10000
        t_current = 10100

        v_ref = np.array([1, 0, -1]).T
        v_meas = np.array([0.5, 0, 100]).T

        gains = PIDFFGains(kP = 100, kI = 1, kD = 10, kFF = 5, i_min = -10, i_max = 10)
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

    def test_mat_gains(self):
        import numpy as np
        t0 = 10000
        t_current = 10100

        v_ref = np.array([[1, 0, -1]]).T
        v_meas = np.array([[0.5, 3, 100]]).T

        # Vectors of gains
        v_kP = [100, 200, 300]
        v_kI = [1, 2, 3]
        v_kD = [10, 20, 30]
        v_kFF = [5, 10, 15]
        i_min = -10
        i_max = 10

        # Matrix gain
        mat_gains = PIDFFGains(kP = np.diag(v_kP), kI = np.diag(v_kI), kD = np.diag(v_kD), kFF = np.diag(v_kFF), i_min = i_min, i_max = i_max)
        mat_pidff = PIDFF(mat_gains, t0)

        control_vals_mat_pidff = mat_pidff.update(t_current, v_ref, v_meas)

        control_vals_scalar_pidffs = []
        for i in range(3):
            # Generate controller for given index
            gains_i = PIDFFGains(kP=v_kP[i], kI=v_kI[i], kD=v_kD[i], kFF=v_kFF[i], i_min=i_min, i_max=i_max)
            pidff_i = PIDFF(gains_i, t0)

            # Control
            control_val_i = pidff_i.update(t_current, v_ref[i], v_meas[i])
            control_vals_scalar_pidffs.append(control_val_i)

        print(f"\nMatrix PIDFF control vals: {control_vals_mat_pidff}")
        print(f"\nScalar PIDFF control vals: {control_vals_scalar_pidffs}")
        # Check that the vector version is the same as the scalar version
        np.testing.assert_allclose(control_vals_mat_pidff, control_vals_scalar_pidffs)


if __name__ == '__main__':
    unittest.main()
