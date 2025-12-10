#!/usr/bin/env python3
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class UR5eCartesianWrapper:
    """
    Minimal wrapper to command end-effector motion through
    /scaled_pos_joint_trajectory_controller
    """

    def __init__(self, joint_names, action_ns):
        self.joint_names = joint_names
        self.client = actionlib.SimpleActionClient(
            action_ns, FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for joint trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to scaled_pos_joint_trajectory controller")

    def send_joint_target(self, positions, duration=0.5):
        if len(positions) != len(self.joint_names):
            raise ValueError("Incorrect joint dimension for command")

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()

        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)

        traj.points.append(point)
        goal.trajectory = traj

        self.client.send_goal(goal)
        self.client.wait_for_result()
