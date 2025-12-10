# -*- coding: utf-8 -*-
"""
@author: smahmou-admin
"""

#!/usr/bin/env python3
import numpy as np
import os
import yaml
import rospy


class RegionModel:
    def __init__(self, A, B, center):
        self.A = A
        self.B = B
        self.center = center


class KoopmanRegionLoader:
    """
    Loads all Koopman operators from config/controller.yaml.
    K matrices are .npz files containing A and B.
    """

    def __init__(self, config_path):
        if not os.path.exists(config_path):
            raise FileNotFoundError("Cannot find controller config: " + config_path)

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.regions = []

        base = os.path.dirname(config_path)
        koop_dir = os.path.join(base, "koopman_ops")

        for region in cfg["regions"]:
            kfile = os.path.join(koop_dir, region["file"])
            if not os.path.exists(kfile):
                rospy.logwarn("Missing Koopman operator file: " + kfile)
                continue

            data = np.load(kfile)
            A = data["A"]
            B = data["B"]

            center = np.array(region["center"])

            self.regions.append(RegionModel(A, B, center))

    def num_regions(self):
        return len(self.regions)

    def find_closest_region(self, x):
        """
        Select nearest region center.
        """
        d = [np.linalg.norm(x - r.center) for r in self.regions]
        idx = int(np.argmin(d))
        return idx, self.regions[idx]
