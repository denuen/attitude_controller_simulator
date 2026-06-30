import math
import unittest

import numpy as np

from visualization.AttitudeMathUtils import AttitudeMathUtils as M


class RotationMatrixTests(unittest.TestCase):

    def test_rotation_matrix_is_identity_at_zero_euler_angles(self):
        np.testing.assert_allclose(M.rotation_matrix_xyz(0.0, 0.0, 0.0),np.eye(3), atol=1e-12)

    def test_rotation_matrix_remains_orthonormal_with_unit_determinant(self):
        for roll, pitch, yaw in [(0.1, -0.2, 0.3), (1.0, 0.5, -0.7), (-2.0, 0.9, 1.4)]:
            r = M.rotation_matrix_xyz(roll, pitch, yaw)
            np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.0, places=12)

    def test_rotation_matrix_matches_explicit_321_yaw_pitch_roll_product(self):
        roll, pitch, yaw = 0.3, -0.4, 0.5
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        np.testing.assert_allclose(M.rotation_matrix_xyz(roll, pitch, yaw),rz @ ry @ rx, atol=1e-12)

    def test_yaw_rotation_maps_body_x_into_inertial_y(self):
        r = M.rotation_matrix_xyz(0.0, 0.0, math.pi / 2.0)
        np.testing.assert_allclose(r @ np.array([1.0, 0.0, 0.0]),[0.0, 1.0, 0.0], atol=1e-12)

    def test_roll_rotation_maps_body_y_into_inertial_z(self):
        r = M.rotation_matrix_xyz(math.pi / 2.0, 0.0, 0.0)
        np.testing.assert_allclose(r @ np.array([0.0, 1.0, 0.0]),[0.0, 0.0, 1.0], atol=1e-12)


class SettlingIndexTests(unittest.TestCase):

    def test_constant_angle_signal_reports_settling_at_index_zero(self):
        self.assertEqual(M.settling_index(np.zeros(100)), 0)

    def test_step_response_enters_settling_band_before_tail(self):
        values = np.concatenate([np.linspace(1.0, 0.0, 30), np.zeros(70)])
        idx = M.settling_index(values, settle_fraction=0.02, floor=1e-6)
        self.assertGreater(idx, 0)
        self.assertLessEqual(idx, 30)
        self.assertTrue(np.all(np.abs(values[idx:]) <= 0.02))

    def test_decaying_oscillation_settles_before_end_of_log(self):
        t = np.linspace(0.0, 10.0, 500)
        values = np.exp(-0.8 * t) * np.cos(6.0 * t)
        idx = M.settling_index(values, settle_fraction=0.02, floor=1e-4)
        self.assertGreater(idx, 0)
        self.assertLess(idx, len(values) - 1)

    def test_monotonic_ramp_never_enters_settling_band(self):
        values = np.linspace(0.0, 10.0, 200)
        self.assertEqual(M.settling_index(values, settle_fraction=0.001, floor=1e-9), -1)

    def test_empty_array_returns_minus_one_settling_index(self):
        self.assertEqual(M.settling_index(np.array([])), -1)


class AngleConversionTests(unittest.TestCase):

    def test_rad_to_deg_converts_pi_to_180_degrees(self):
        self.assertAlmostEqual(M.rad_to_deg(math.pi), 180.0)
        self.assertAlmostEqual(M.rad_to_deg(math.pi / 2.0), 90.0)

    def test_degree_radian_round_trip_is_exact(self):
        for deg in [0.0, 45.0, -123.4, 360.0]:
            self.assertAlmostEqual(M.rad_to_deg(M.deg_to_rad(deg)), deg, places=10)

    def test_rad_to_deg_accepts_numpy_array_input(self):
        np.testing.assert_allclose(M.rad_to_deg(np.array([0.0, math.pi])),[0.0, 180.0])


if __name__ == "__main__":
    unittest.main()
