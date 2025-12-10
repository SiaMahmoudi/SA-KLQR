#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
import sensor_msgs.point_cloud2 as pc2
import yaml
import os


class PlaneDetector:
    def __init__(self):
        rospy.init_node("plane_detector")

        # Load config
        cfg_path = rospy.get_param("~config", "")
        if cfg_path == "":
            raise RuntimeError("plane.yaml path not provided")

        if not os.path.exists(cfg_path):
            raise RuntimeError("Cannot find plane config: " + cfg_path)

        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.cloud_topic = cfg["cloud_topic"]
        self.distance_threshold = cfg["distance_threshold"]
        self.ransac_n = cfg["ransac_n"]
        self.num_iter = cfg["num_iterations"]
        self.voxel = cfg["voxel_size"]

        rospy.loginfo("PlaneDetector: subscribing to " + self.cloud_topic)

        # Publishers
        self.pub_tilt = rospy.Publisher("/surface_tilt", Vector3Stamped, queue_size=5)
        self.pub_normal = rospy.Publisher("/surface_normal", Vector3Stamped, queue_size=5)

        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cb_cloud)

        rospy.loginfo("PlaneDetector initialized.")

    def cb_cloud(self, cloud_msg):
        # Convert ROS PointCloud2 → numpy Nx3
        pts = []
        for p in pc2.read_points(cloud_msg, field_names=("x","y","z"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])

        if len(pts) < 500:
            return

        pts = np.array(pts, dtype=np.float32)

        # Make Open3D cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        # Downsample
        pcd = pcd.voxel_down_sample(self.voxel)

        if len(pcd.points) < 100:
            return

        # RANSAC plane detection
        try:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=self.ransac_n,
                num_iterations=self.num_iter
            )
        except:
            return

        a, b, c, d = plane_model
        normal = np.array([a, b, c], dtype=float)

        # Normalize
        if np.linalg.norm(normal) < 1e-6:
            return
        normal = normal / np.linalg.norm(normal)

        # Extract tilt_x, tilt_y
        # tilt relative to camera z-axis
        tilt_x = normal[0]
        tilt_y = normal[1]

        # Publish /surface_tilt
        tmsg = Vector3Stamped()
        tmsg.header.stamp = rospy.Time.now()
        tmsg.header.frame_id = "camera_link"
        tmsg.vector.x = tilt_x
        tmsg.vector.y = tilt_y
        tmsg.vector.z = 0.0
        self.pub_tilt.publish(tmsg)

        # Publish /surface_normal
        nmsg = Vector3Stamped()
        nmsg.header = tmsg.header
        nmsg.vector.x = normal[0]
        nmsg.vector.y = normal[1]
        nmsg.vector.z = normal[2]
        self.pub_normal.publish(nmsg)


if __name__ == "__main__":
    node = PlaneDetector()
    rospy.spin()
