import numpy as np
from .VisualizationConfig import VisualizationConfig
import logging
from .AttitudeMathUtils import AttitudeMathUtils
from typing import Dict, List, Optional

# Builds and transforms the rectangular body mesh for 3D rendering
class RigidBodyGeometry:

    def __init__(self, config: VisualizationConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)

        self._base_vertices = None
        self._faces = None

    # 8 body vertices centered at the origin, cached
    def get_body_vertices(self) -> np.ndarray:
        if self._base_vertices is None:
            half_length: float = self.config.body_length / 2.0
            half_width: float = self.config.body_width / 2.0
            half_height: float = self.config.body_height / 2.0

            self._base_vertices = np.array([
                [-half_length, -half_width, -half_height],  # 0: back-left-bottom
                [+half_length, -half_width, -half_height],  # 1: front-left-bottom
                [+half_length, +half_width, -half_height],  # 2: front-right-bottom
                [-half_length, +half_width, -half_height],  # 3: back-right-bottom
                [-half_length, -half_width, +half_height],  # 4: back-left-top
                [+half_length, -half_width, +half_height],  # 5: front-left-top
                [+half_length, +half_width, +half_height],  # 6: front-right-top
                [-half_length, +half_width, +half_height],  # 7: back-right-top
            ], dtype=np.float64)

        return self._base_vertices.copy()

    # Six quad faces as vertex-index lists, cached
    def get_body_faces(self) -> List[List[int]]:
        if self._faces is None:
            self._faces = [
                [0, 1, 2, 3],  # Bottom
                [4, 7, 6, 5],  # Top
                [0, 4, 5, 1],  # Left
                [2, 6, 7, 3],  # Right
                [0, 3, 7, 4],  # Back
                [1, 5, 6, 2],  # Front
            ]

        return self._faces

    # Rotate by (roll, pitch, yaw), then translate to position
    def transform_body_vertices(self, roll: float, pitch: float, yaw: float, position: Optional[np.ndarray] = None) -> np.ndarray:
        if position is None:
            position = np.array([0.0, 0.0, 0.0])

        vertices = self.get_body_vertices()
        rotation_matrix = AttitudeMathUtils.rotation_matrix_xyz(roll, pitch, yaw)

        rotated_vertices = (rotation_matrix @ vertices.T).T

        return rotated_vertices + position

    # Body-fixed axes rotated into the inertial frame, as {'x': [start, end], ...}
    def create_coordinate_axes(self, roll: float, pitch: float, yaw: float, position: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        if position is None:
            position = np.array([0.0, 0.0, 0.0])

        rotation_matrix = AttitudeMathUtils.rotation_matrix_xyz(roll, pitch, yaw)
        scale = self.config.coordinate_axis_scale

        body_x = np.array([scale, 0.0, 0.0])
        body_y = np.array([0.0, scale, 0.0])
        body_z = np.array([0.0, 0.0, scale])

        inertial_x = rotation_matrix @ body_x
        inertial_y = rotation_matrix @ body_y
        inertial_z = rotation_matrix @ body_z

        return {
            'x': np.array([position, position + inertial_x]),
            'y': np.array([position, position + inertial_y]),
            'z': np.array([position, position + inertial_z])
        }
