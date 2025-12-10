#!/usr/bin/env python3
import rospy
import csv
import os
import datetime

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState


class DataLogger:
    def __init__(self):
        rospy.init_node("data_logger")

        # Create output directory
        base_dir = os.path.expanduser("~/sa_klqr_logs")
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.outdir = os.path.join(base_dir, timestamp)
        os.makedirs(self.outdir)

        self.csv_path = os.path.join(self.outdir, "swab_data.csv")

        rospy.loginfo("Logging data to: " + self.csv_path)

        # Open CSV file
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # CSV header
        self.writer.writerow([
            "time",

            # FSR
            "force_total",
            "cx",
            "cy",

            # Tilt
            "tilt_x",
            "tilt_y",

            # KLQR state vector
            "x0", "x1", "x2", "x3", "x4", "x5",

            # KLQR output
            "u",

            # Joint positions
            "jp0", "jp1", "jp2", "jp3", "jp4", "jp5"
        ])

        # Data placeholders
        self.force_total = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.tilt_x = 0.0
        self.tilt_y = 0.0
        self.state_vec = [0]*6
        self.u = 0.0
        self.joints = [0]*6

        # Subscribers
        rospy.Subscriber("/fsr/force_total", Float32, self.cb_force)
        rospy.Subscriber("/fsr/force_map", Float32MultiArray, self.cb_centroid)
        rospy.Subscriber("/surface_tilt", Vector3Stamped, self.cb_tilt)
        rospy.Subscriber("/joint_states", JointState, self.cb_joints)

        rospy.Subscriber("/sa_klqr/state_vector", Float32MultiArray, self.cb_state)
        rospy.Subscriber("/sa_klqr/control_output", Float32, self.cb_u)

    # -----------------------------------
    # Callbacks
    # -----------------------------------
    def cb_force(self, msg):
        self.force_total = msg.data

    def cb_centroid(self, msg):
        import numpy as np
        m = np.array(msg.data).reshape((8,8))
        tot = m.sum()
        if tot > 1e-6:
            xs = np.arange(8)
            ys = np.arange(8)
            self.cx = float((xs[:,None] * m).sum() / tot)
            self.cy = float((ys[None,:] * m).sum() / tot)

    def cb_tilt(self, msg):
        self.tilt_x = msg.vector.x
        self.tilt_y = msg.vector.y

    def cb_state(self, msg):
        self.state_vec = list(msg.data)

    def cb_u(self, msg):
        self.u = msg.data

    def cb_joints(self, msg):
        jp = dict(zip(msg.name, msg.position))
        names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.joints = [jp.get(n, 0.0) for n in names]

    # -----------------------------------
    # Run Loop
    # -----------------------------------
    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()

            row = [
                now,
                self.force_total,
                self.cx,
                self.cy,
                self.tilt_x,
                self.tilt_y,
                *self.state_vec,
                self.u,
                *self.joints
            ]

            self.writer.writerow(row)
            rate.sleep()


if __name__ == "__main__":
    node = DataLogger()
    node.run()
