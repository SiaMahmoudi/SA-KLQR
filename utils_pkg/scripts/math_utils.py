#!/usr/bin/env python3
import numpy as np


def normalize(v):
    """ Normalize a vector safely. """
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros_like(v)
    return v / n


def saturate(x, min_val, max_val):
    """ Clamp x into [min_val, max_val]. """
    return float(np.clip(x, min_val, max_val))


def wrap_angle(theta):
    """ Wrap angle to [-pi, pi]. """
    return (theta + np.pi) % (2 * np.pi) - np.pi


def moving_average(data, window=5):
    """ Simple moving average smoother. """
    if len(data) < window:
        return np.mean(data)
    return np.mean(data[-window:])
