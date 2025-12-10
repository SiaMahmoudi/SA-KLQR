#!/usr/bin/env python3
import numpy as np


def compute_centroid(force_map):
    """
    force_map: flattened or (N,N) array of pressures

    Returns:
        cx, cy
    """
    fm = np.array(force_map).reshape((8, 8))  # adjust if pad not 8x8

    total = np.sum(fm)
    if total < 1e-6:
        return 0.0, 0.0

    nx, ny = fm.shape

    xs = np.arange(nx)
    ys = np.arange(ny)

    cx = np.sum(xs[:, None] * fm) / total
    cy = np.sum(ys[None, :] * fm) / total

    return float(cx), float(cy)


def fuzzy_entropy(signal, m=2, r=0.2):
    """
    Simple fuzzy entropy measure for centroid stability.
    Not critical for control but may help diagnose noise.
    """

    signal = np.array(signal, dtype=float)
    N = len(signal)
    if N < m + 1:
        return 0.0

    def phi(m):
        vals = []
        for i in range(N - m):
            template = signal[i:i+m]
            diffs = np.abs(signal[i+1:i+1+m] - template)
            vals.append(np.exp(-(diffs / r) ** 2).mean())
        return np.mean(vals)

    return -np.log(phi(m+1) / phi(m))
