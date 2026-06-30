import logging
import numpy as np
import pandas as pd
from typing import Tuple, Optional, Union
from pathlib import Path
from .VisualizationConfig import ValidationResult

# Loads and validates the CSV exported by the C++ simulator
class SimulationDataLoader:

    # Written by SimulationManager::exportLog
    REQUIRED_COLUMNS = {
        'time', 'pitch', 'yaw', 'roll',
        'omega_x', 'omega_y', 'omega_z'
    }

    ATTITUDE_LIMITS = {
        'roll': (-np.pi, np.pi),
        'pitch': (-np.pi, np.pi),
        'yaw': (-np.pi, np.pi),
        'omega_x': (-10.0, 10.0),
        'omega_y': (-10.0, 10.0),
        'omega_z': (-10.0, 10.0)
    }

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self._data = None
        self._data_file_path = None

    # On success returns (VALID, frame), else (error code, None)
    def load_data(self, file_path: Union[str, Path]) -> Tuple[ValidationResult, Optional[pd.DataFrame]]:
        file_path = Path(file_path)

        if not file_path.exists():
            self.logger.error(f"Data file not found: {file_path}")
            self.logger.info("Please run the simulator first: ./attitude_simulator")
            return ValidationResult.INVALID_FORMAT, None

        try:
            data_frame = pd.read_csv(file_path)
            self.logger.info(f"Loaded {len(data_frame)} data points from {file_path}")

            validation_result = self._validate_data(data_frame)

            if validation_result == ValidationResult.VALID:
                self._data = data_frame
                self._data_file_path = file_path

            return validation_result, data_frame if validation_result == ValidationResult.VALID else None

        except pd.errors.EmptyDataError:
            self.logger.error(f"Empty data file: {file_path}")
            return ValidationResult.EMPTY_DATA, None

        except Exception as e:
            self.logger.error(f"Error loading data file {file_path}: {e}")
            return ValidationResult.INVALID_FORMAT, None

    # Structure, ranges and time monotonicity
    def _validate_data(self, data_frame: pd.DataFrame) -> ValidationResult:
        if data_frame.empty:
            self.logger.error("Data frame is empty")
            return ValidationResult.EMPTY_DATA

        missing_columns = self.REQUIRED_COLUMNS - set(data_frame.columns)
        if missing_columns:
            self.logger.error(f"Missing required columns: {missing_columns}")
            return ValidationResult.MISSING_COLUMNS

        for column, (min_val, max_val) in self.ATTITUDE_LIMITS.items():
            if column in data_frame.columns:
                out_of_bounds = (
                    (data_frame[column] < min_val) |
                    (data_frame[column] > max_val)
                ).any()

                if out_of_bounds:
                    self.logger.warning(f"{column}: values outside the expected range")

        if 'time' in data_frame.columns:
            if not data_frame['time'].is_monotonic_increasing:
                self.logger.warning("time is not monotonic increasing")

        self.logger.info("Validation passed")
        return ValidationResult.VALID

    @property
    def data(self) -> Optional[pd.DataFrame]:
        return self._data

    @property
    def data_file_path(self) -> Optional[Path]:
        return self._data_file_path
