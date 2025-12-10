#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3Stamped


class SwabManager:
    """
    Oversees the swabbing pipeline:
      1. Wait for FSR to stabilize
      2. Wait for plane normal detection
      3. Enable SA-KLQR controller logic
    """

    def __init__(self):
        rospy.init_node("swab_manager")

        self.force_ready = False
        self.tilt_ready = False
        self.current_force = 0.0
        self.tilt_x = 0.0
        self.tilt_y = 0.0

        # Status publishers
        self.pub_enable = rospy.Publisher("/sa_klqr/enable", Bool, queue_size=2)

        # Subscribers
        rospy.Subscriber("/fsr/force_total", Float32, self.cb_force)
        rospy.Subscriber("/surface_tilt", Vector3Stamped, self.cb_tilt)

        rospy.loginfo("Swab Manager started.")

    # -------------------------------
    # Callbacks
    # -------------------------------
    def cb_force(self, msg):
        self.current_force = msg.data

        # Basic rule: force has to be > 0.5N to start
        if self.current_force > 0.5:
            self.force_ready = True

    def cb_tilt(self, msg):
        self.tilt_x = msg.vector.x
        self.tilt_y = msg.vector.y

        # Tilt detection simply means plane normal is non-zero
        if abs(self.tilt_x) > 1e-4 or abs(self.tilt_y) > 1e-4:
            self.tilt_ready = True

    # -------------------------------
    # Main Loop
    # -------------------------------
    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.force_ready and self.tilt_ready:
                self.pub_enable.publish(True)
            else:
                self.pub_enable.publish(False)

            rate.sleep()


if __name__ == "__main__":
    node = SwabManager()
    node.run()
