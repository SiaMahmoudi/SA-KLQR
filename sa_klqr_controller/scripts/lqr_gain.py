#!/usr/bin/env python3
import numpy as np
from scipy.linalg import solve_continuous_are


def compute_lqr_gain(A, B, Q, R):
    """
    Computes the continuous-time LQR gain:
        K = R^{-1} B^T P
    where P solves the CARE:
        A^T P + P A - P B R^{-1} B^T P + Q = 0
    """

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K
