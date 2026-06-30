import os

os.environ.setdefault("MPLBACKEND", "Agg")

import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from visualization.VisualizationConfig import VisualizationConfig
from visualization.AttitudeVisualizer import AttitudeVisualizer


def make_csv(path, rows=60):
    t = np.linspace(0.0, 1.0, rows)
    pd.DataFrame({
        "time": t,
        "pitch": 0.10 * np.sin(t),
        "yaw": 0.05 * np.cos(t),
        "roll": 0.08 * np.sin(2.0 * t),
        "omega_x": 0.01 * np.cos(t),
        "omega_y": -0.01 * np.sin(t),
        "omega_z": 0.005 * np.ones(rows),
    }).to_csv(path, index=False)


class VisualizerTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._show = plt.show
        plt.show = lambda *args, **kwargs: None

    @classmethod
    def tearDownClass(cls):
        plt.show = cls._show

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = Path(self.tmp.name)
        self.csv = self.out / "sim.csv"
        make_csv(self.csv)

        self.config = VisualizationConfig()
        self.config.output_dir = str(self.out)
        self.viz = AttitudeVisualizer(self.config)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_load_valid_simulation_csv_populates_dataframe(self):
        self.assertTrue(self.viz.load_simulation_data(self.csv))
        self.assertIsNotNone(self.viz.simulation_data)

    def test_render_methods_return_false_when_no_csv_loaded(self):
        self.assertFalse(self.viz.plot_static_sequence())
        self.assertFalse(self.viz.create_trajectory_plot())
        self.assertFalse(self.viz.create_animated_plot())

    def test_static_sequence_export_writes_configured_png(self):
        self.viz.load_simulation_data(self.csv)
        self.assertTrue(self.viz.plot_static_sequence(num_frames=5))
        self.assertTrue((self.out / self.config.static_plot_filename).exists())

    def test_trajectory_export_writes_configured_png(self):
        self.viz.load_simulation_data(self.csv)
        self.assertTrue(self.viz.create_trajectory_plot())
        self.assertTrue((self.out / self.config.trajectory_filename).exists())

    def test_animation_export_writes_configured_mp4(self):
        self.viz.load_simulation_data(self.csv)
        self.assertTrue(self.viz.create_animated_plot())
        self.assertTrue((self.out / self.config.animation_filename).exists())

    def test_settling_detector_finds_transient_end_on_decaying_signal(self):
        rows = 400
        t = np.linspace(0.0, 8.0, rows)
        envelope = np.exp(-1.5 * t)
        path = self.out / "decay.csv"
        pd.DataFrame({
            "time": t,
            "pitch": 0.20 * envelope * np.cos(5.0 * t),
            "yaw": 0.15 * envelope * np.sin(5.0 * t),
            "roll": 0.18 * envelope,
            "omega_x": 0.30 * envelope,
            "omega_y": -0.25 * envelope,
            "omega_z": 0.20 * envelope,
        }).to_csv(path, index=False)

        self.viz.load_simulation_data(path)
        result = self.viz.compute_settling()
        self.assertTrue(result["settled"])
        self.assertGreater(result["index"], 0)
        self.assertLess(result["index"], rows - 1)
        self.assertGreater(result["time"], 0.0)
        self.assertLess(result["time"], t[-1])


if __name__ == "__main__":
    unittest.main()
