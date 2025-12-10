#!/usr/bin/env python3
import numpy as np


class StateBuilder:
    """
    Builds a 6-dimensional state vector for SA-KLQR:
    
        x = [
            force_error,
            centroid_x_error,
            centroid_y_error,
            tilt_x,
            tilt_y,
            force_derivative
        ]
    """

    def __init__(self, alpha_force=1.0):
        self.prev_force = None
        self.alpha_force = alpha_force  # smoothing factor for derivative

    def compute_state(self, force, target_force, cx, cy, tilt_x, tilt_y):
        # force error
        fe = float(force - target_force)

        # centroid errors (already aligned to robot frame in your system)
        cx_err = float(cx)
        cy_err = float(cy)

        # tilt from plane_estimator
        tx = float(tilt_x)
        ty = float(tilt_y)

        # force derivative
        if self.prev_force is None:
            f_d = 0.0
        else:
            f_d = float(force - self.prev_force)

        self.prev_force = force

        x = np.array([fe, cx_err, cy_err, tx, ty, f_d], dtype=float)
        return x
