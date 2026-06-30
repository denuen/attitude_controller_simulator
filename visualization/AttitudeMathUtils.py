import numpy as np
from typing import Union

# 3-2-1 Euler-angle helpers, matching the C++ simulator's logged attitude
class AttitudeMathUtils:

    # Body-to-inertial rotation, Rz(yaw) @ Ry(pitch) @ Rx(roll) expanded inline
    @staticmethod
    def rotation_matrix_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ], dtype=np.float64)

    # First index staying within the settle band of the tail-mean value, else -1
    @staticmethod
    def settling_index(values: np.ndarray, settle_fraction: float = 0.02,
                       tail_fraction: float = 0.05, floor: float = 1e-3) -> int:
        values = np.asarray(values, dtype=np.float64)
        n = values.size
        if n == 0:
            return -1

        tail = max(1, int(round(n * tail_fraction)))
        final = float(values[-tail:].mean())

        excursion = float(np.max(np.abs(values - final)))
        band = max(settle_fraction * excursion, floor)

        within = np.abs(values - final) <= band
        outside = np.where(~within)[0]
        if outside.size == 0:
            return 0
        last_outside = int(outside[-1])
        if last_outside >= n - 1:
            return -1

        return last_outside + 1

    # rad in the simulator, deg in plots and telemetry
    @staticmethod
    def rad_to_deg(angle_rad: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        return angle_rad * 180.0 / np.pi

    @staticmethod
    def deg_to_rad(angle_deg: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        return angle_deg * np.pi / 180.0
