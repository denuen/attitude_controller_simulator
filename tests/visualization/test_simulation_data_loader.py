import logging
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from visualization.SimulationDataLoader import SimulationDataLoader
from visualization.VisualizationConfig import ValidationResult

# Header written by SimulationManager::exportLog
CPP_HEADER = ["time", "pitch", "yaw", "roll", "omega_x", "omega_y", "omega_z"]


def make_frame(rows=10):
    t = np.linspace(0.0, 0.05, rows)
    return pd.DataFrame({
        "time": t,
        "pitch": np.zeros(rows),
        "yaw": np.zeros(rows),
        "roll": np.zeros(rows),
        "omega_x": np.zeros(rows),
        "omega_y": np.zeros(rows),
        "omega_z": np.zeros(rows),
    })


class DataLoaderTests(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.loader = SimulationDataLoader(logging.getLogger("test.loader"))

    def tearDown(self):
        self.tmp.cleanup()

    def _write(self, frame, name="data.csv"):
        path = self.dir / name
        frame.to_csv(path, index=False)
        return path

    def test_required_columns_match_simulation_manager_export_log(self):
        self.assertEqual(SimulationDataLoader.REQUIRED_COLUMNS, set(CPP_HEADER))

    def test_pitch_validation_span_is_minus_pi_to_plus_pi(self):
        lo, hi = SimulationDataLoader.ATTITUDE_LIMITS["pitch"]
        self.assertAlmostEqual(lo, -np.pi)
        self.assertAlmostEqual(hi, np.pi)

    def test_valid_kinematic_csv_returns_VALID_and_stores_dataframe(self):
        path = self._write(make_frame())
        result, frame = self.loader.load_data(path)
        self.assertEqual(result, ValidationResult.VALID)
        self.assertIsNotNone(frame)
        self.assertEqual(len(self.loader.data), 10)
        self.assertEqual(self.loader.data_file_path, path)

    def test_missing_file_returns_INVALID_FORMAT(self):
        result, frame = self.loader.load_data(self.dir / "nope.csv")
        self.assertEqual(result, ValidationResult.INVALID_FORMAT)
        self.assertIsNone(frame)

    def test_missing_required_column_returns_MISSING_COLUMNS(self):
        frame = make_frame().drop(columns=["omega_z"])
        result, _ = self.loader.load_data(self._write(frame))
        self.assertEqual(result, ValidationResult.MISSING_COLUMNS)

    def test_empty_csv_returns_EMPTY_DATA(self):
        path = self.dir / "empty.csv"
        path.write_text("")
        result, _ = self.loader.load_data(path)
        self.assertEqual(result, ValidationResult.EMPTY_DATA)

    def test_out_of_range_pitch_warns_but_still_returns_VALID(self):
        frame = make_frame()
        frame.loc[0, "pitch"] = 4.0  # beyond pi
        with self.assertLogs("test.loader", level="WARNING") as logs:
            result, _ = self.loader.load_data(self._write(frame))
        self.assertEqual(result, ValidationResult.VALID)
        self.assertTrue(any("pitch" in line for line in logs.output))

    def test_non_monotonic_time_column_emits_warning(self):
        frame = make_frame()
        frame.loc[5, "time"] = -1.0
        with self.assertLogs("test.loader", level="WARNING") as logs:
            self.loader.load_data(self._write(frame))
        self.assertTrue(any("monotonic" in line.lower() for line in logs.output))


if __name__ == "__main__":
    unittest.main()
