import unittest

import numpy as np

from visualization.VisualizationConfig import VisualizationConfig
from visualization.RigidBodyGeometry import RigidBodyGeometry


class GeometryTests(unittest.TestCase):

    def setUp(self):
        self.config = VisualizationConfig()
        self.geom = RigidBodyGeometry(self.config)

    def test_body_vertices_are_eight_points_centered_at_origin(self):
        v = self.geom.get_body_vertices()
        self.assertEqual(v.shape, (8, 3))
        np.testing.assert_allclose(v.mean(axis=0), [0.0, 0.0, 0.0], atol=1e-12)

    def test_body_vertices_span_configured_half_dimensions(self):
        v = self.geom.get_body_vertices()
        np.testing.assert_allclose(np.abs(v).max(axis=0),
                                   [self.config.body_length / 2.0,
                                    self.config.body_width / 2.0,
                                    self.config.body_height / 2.0])

    def test_get_body_vertices_returns_defensive_copy(self):
        v = self.geom.get_body_vertices()
        v[0, 0] = 999.0
        self.assertNotEqual(self.geom.get_body_vertices()[0, 0], 999.0)

    def test_body_mesh_has_six_quad_faces_with_valid_indices(self):
        faces = self.geom.get_body_faces()
        self.assertEqual(len(faces), 6)
        for face in faces:
            self.assertEqual(len(face), 4)
            self.assertTrue(all(0 <= i < 8 for i in face))
        self.assertIs(self.geom.get_body_faces(), faces)

    def test_zero_attitude_translation_adds_position_offset_only(self):
        pos = np.array([1.0, -2.0, 3.0])
        base = self.geom.get_body_vertices()
        moved = self.geom.transform_body_vertices(0.0, 0.0, 0.0, pos)
        np.testing.assert_allclose(moved, base + pos, atol=1e-12)

    def test_rotation_preserves_edge_length_under_euler_transform(self):
        base = self.geom.get_body_vertices()
        rotated = self.geom.transform_body_vertices(0.5, -0.3, 1.2)
        self.assertAlmostEqual(np.linalg.norm(base[0] - base[1]),
                               np.linalg.norm(rotated[0] - rotated[1]), places=12)

    def test_body_axes_at_identity_match_scaled_unit_vectors(self):
        axes = self.geom.create_coordinate_axes(0.0, 0.0, 0.0)
        self.assertEqual(set(axes), {"x", "y", "z"})
        scale = self.config.coordinate_axis_scale
        for name, expected_end in zip("xyz", np.eye(3) * scale):
            segment = axes[name]
            np.testing.assert_allclose(segment[0], [0.0, 0.0, 0.0], atol=1e-12)
            np.testing.assert_allclose(segment[1], expected_end, atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(segment[1] - segment[0]),scale, places=12)


if __name__ == "__main__":
    unittest.main()
