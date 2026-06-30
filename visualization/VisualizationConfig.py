from typing import Dict
from enum import Enum

class VisualizationType(Enum):
    STATIC_SEQUENCE = 1
    ANIMATED_3D = 2
    TRAJECTORY_3D = 3
    ALL_VISUALIZATIONS = 4

class ValidationResult(Enum):
    VALID = "VALID"
    INVALID_FORMAT = "INVALID_FORMAT"
    MISSING_COLUMNS = "MISSING_COLUMNS"
    EMPTY_DATA = "EMPTY_DATA"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"

# Tunable parameters for the visualization system
class VisualizationConfig:

    def __init__(self):
        self.figure_dpi: int = 300
        self.animation_fps: int = 100
        self.animation_interval: int = 100
        self.animation_dpi: int = 100

        # Settling detection, from the CSV signals only
        self.settling_fraction: float = 0.02   # band as a fraction of the initial excursion
        self.settling_tail_fraction: float = 0.05  # tail used to estimate the final value
        self.settling_angle_floor: float = 1e-3    # rad, minimum angle band
        self.settling_rate_floor: float = 1e-3     # rad/s, minimum rate band
        # Extra frames kept after settling so the steady state stays visible
        self.settling_margin_fraction: float = 0.05
        self.settling_margin_min_frames: int = 10

        # Body dimensions (m)
        self.body_length: float = 2.0
        self.body_width: float = 1.0
        self.body_height: float = 0.5

        self.coordinate_axis_scale: float = 1.5
        # Peak rate maps to this length; arrow stays inside the [-2, 2] cube
        self.omega_arrow_max_len: float = 1.8
        # Each torque arrow is normalized to its own peak (magnitudes differ a lot)
        self.torque_arrow_max_len: float = 1.5
        # Display-only moving average over the torque arrows; 1 disables it
        self.torque_smoothing_window: int = 15
        # Kept off the red/green/blue body axes so arrows don't blend in
        self.omega_arrow_color: str = '#ff2eff'
        self.control_torque_color: str = '#ff8c00'
        self.gyro_torque_color: str = '#00e5ff'
        # Internal term, near-zero outside aggressive slews; off by default
        self.show_gyroscopic_arrow: bool = False
        # Reference axes drawn thin/faint so the dynamic vectors dominate
        self.axis_line_width: float = 1.6
        self.axis_line_alpha: float = 0.55
        self.trajectory_line_width: float = 1.5

        self.output_dir: str = "simulation_output"
        self.static_plot_filename: str = "body_orientation_sequence.png"
        self.animation_filename: str = "body_animation.mp4"
        self.trajectory_filename: str = "attitude_trajectory_3d.png"

        self.body_face_color: str = 'lightsteelblue'
        self.body_edge_color: str = 'darkblue'
        # X=roll (red), Y=pitch (green), Z=yaw (blue)
        self.axis_colors: Dict[str, str] = {
            'x': '#d62728',
            'y': '#2ca02c',
            'z': '#1f77b4'
        }
