#!/usr/bin/env python3
import numpy as np


class LowPassFilter:
    def __init__(self, alpha=0.1, initial_value=0.0):
        self.alpha = alpha
        self.value = initial_value
        self.initialized = False

    def reset(self, val=0.0):
        self.value = val
        self.initialized = False

    def update(self, x):
        if not self.initialized:
            self.value = x
            self.initialized = True
        else:
            self.value = self.alpha * x + (1 - self.alpha) * self.value
        return self.value


class DerivativeFilter:
    """
    Computes dx/dt with simple low-pass filtering.
    """

    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev = None
        self.filtered = 0.0

    def update(self, x):
        if self.prev is None:
            self.prev = x
            return 0.0

        raw = x - self.prev
        self.filtered = self.alpha * raw + (1 - self.alpha) * self.filtered
        self.prev = x
        return self.filtered


class SlidingWindowFilter:
    def __init__(self, size=10):
        self.size = size
        self.data = []

    def update(self, x):
        self.data.append(x)
        if len(self.data) > self.size:
            self.data.pop(0)
        return np.mean(self.data)
