#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState

from koopman_loader import KoopmanRegionLoader
from lqr_gain import compute_lqr_gain
from state_builder import StateBuilder
from centroid_fuzzy import compute_centroid
from cartesian_wrapper import UR5eCartesianWrapper


class SAKLQRController:
    """
    Main SA-KLQR controller node:
      - Builds 6-state vector
      - Selects Koopman region
      - Computes LQR control
      - Blends control
      - Commands robot joint
      - Publishes state + control for logging
      - Enables only when swab_manager allows
    """

    def __init__(self):
        rospy.init_node("sa_klqr_controller")

        # ========================================================
        # Load controller parameters
        # ========================================================
        cfg = rospy.get_param("~controller_config")

        self.target_force = cfg["target_force"]
        self.joint_names = cfg["joint_names"]
        self.control_joint = cfg["control_joint"]

        # LQR weights
        self.Q = np.diag(cfg["Q"])
        self.R = np.diag(cfg["R"])

        # blending factor β
        self.beta = cfg["beta"]

        # joint_scale: how much KLQR output affects joint angle
        self.joint_step = cfg["joint_step"]

        # Koopman loader
        controller_yaml = cfg["controller_yaml"]
        self.koop_loader = KoopmanRegionLoader(controller_yaml)

        # Precompute LQR gains
        self.K_regions = []
        rospy.loginfo("Computing LQR gains for Koopman regions...")
        for region in self.koop_loader.regions:
            K = compute_lqr_gain(region.A, region.B, self.Q, self.R)
            self.K_regions.append(K)
        rospy.loginfo("All LQR gains computed.")

        # State builder (6 components)
        self.state_builder = StateBuilder()

        # Cartesian wrapper for joint control
        self.wrapper = UR5eCartesianWrapper(
            self.joint_names,
            "/scaled_pos_joint_trajectory_controller/follow_joint_trajectory"
        )

        # ========================================================
        # Internal state variables
        # ========================================================
        self.force_map = None
        self.force_total = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.tilt_x = 0.0
        self.tilt_y = 0.0
        self.joint_positions = None

        self.enable_control = False

        # ========================================================
        # Publishers for logging
        # ========================================================
        self.pub_state = rospy.Publisher("/sa_klqr/state_vector", Float32MultiArray, queue_size=5)
        self.pub_u = rospy.Publisher("/sa_klqr/control_output", Float32, queue_size=5)

        # ========================================================
        # ROS Subscribers
        # ========================================================
        rospy.Subscriber("/fsr/force_map", Float32MultiArray, self.cb_fsr)
        rospy.Subscriber("/fsr/force_total", Float32, self.cb_force_total)
        rospy.Subscriber("/surface_tilt", Vector3Stamped, self.cb_tilt)
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_state)

        # Enable signal from swab_manager
        rospy.Subscriber("/sa_klqr/enable", Bool, self.cb_enable)

        rospy.loginfo("SA-KLQR controller is fully initialized.")

    # -----------------------------------------------------
    # Callbacks
    # -----------------------------------------------------
    def cb_enable(self, msg):
        self.enable_control = bool(msg.data)

    def cb_fsr(self, msg):
        self.force_map = np.array(msg.data)
        self.cx, self.cy = compute_centroid(self.force_map)

    def cb_force_total(self, msg):
        self.force_total = msg.data

    def cb_tilt(self, msg):
        self.tilt_x = msg.vector.x
        self.tilt_y = msg.vector.y

    def cb_joint_state(self, msg):
        jp = dict(zip(msg.name, msg.position))
        self.joint_positions = [jp.get(j, 0.0) for j in self.joint_names]

    # -----------------------------------------------------
    # Control loop
    # -----------------------------------------------------
    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            # -----------------------------------------------------
            # Safety / readiness checks
            # -----------------------------------------------------
            if not self.enable_control:
                # publish zeros for logging
                self.pub_state.publish(Float32MultiArray(data=[0]*6))
                self.pub_u.publish(0.0)
                rate.sleep()
                continue

            if self.force_map is None or self.joint_positions is None:
                rate.sleep()
                continue

            # -----------------------------------------------------
            # Build KLQR state vector
            # -----------------------------------------------------
            x = self.state_builder.compute_state(
                force=self.force_total,
                target_force=self.target_force,
                cx=self.cx,
                cy=self.cy,
                tilt_x=self.tilt_x,
                tilt_y=self.tilt_y
            )

            # Publish state for logger
            self.pub_state.publish(Float32MultiArray(data=x.tolist()))

            # -----------------------------------------------------
            # Region selection (nearest region center)
            # -----------------------------------------------------
            region_idx, region = self.koop_loader.find_closest_region(x)
            K = self.K_regions[region_idx]

            # Compute control output
            u = float(-K.dot(x))

            # Blend output
            u_blend = self.beta * u

            # Publish control output for logger
            self.pub_u.publish(u_blend)

            # -----------------------------------------------------
            # Apply joint command
            # -----------------------------------------------------
            q = self.joint_positions.copy()
            jidx = self.joint_names.index(self.control_joint)
            q[jidx] += u_blend * self.joint_step

            try:
                self.wrapper.send_joint_target(q, duration=0.3)
            except Exception as e:
                rospy.logerr("Joint command failed: " + str(e))

            rate.sleep()


if __name__ == "__main__":
    node = SAKLQRController()
    node.run()
