"""Read the challenge track CSV (x_m, y_m, w_tr_right_m, w_tr_left_m)."""

import os
from dataclasses import dataclass

import numpy as np

DEFAULT_TRACK = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "silverstone.csv")


@dataclass(frozen=True)
class Track:
    x: np.ndarray
    y: np.ndarray
    w_right: np.ndarray
    w_left: np.ndarray

    @property
    def points(self):
        """(N, 2) array of centreline points."""
        return np.column_stack([self.x, self.y])


def load_track(path=DEFAULT_TRACK):
    data = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    if data.shape[1] < 4 or data.shape[0] < 3:
        raise ValueError(f"{path}: expected at least 3 rows of 4 columns")
    if not np.all(np.isfinite(data[:, :4])):
        raise ValueError(f"{path}: contains non-numeric values")
    return Track(*data[:, :4].T.copy())


def load_points(path=DEFAULT_TRACK):
    return load_track(path).points
