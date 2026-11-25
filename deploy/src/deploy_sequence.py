#!/usr/bin/env python

import time
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class DeploySequence:

    def __init__(self):

        self.x = None
        self.y = None
        self.z = None
        self.last_pose_time = 0.0

        self.command_pub = rospy.Publisher(
            "/end_effector/command",
            String,
            queue_size=10
        )

        self.pose_sub = rospy.Subscriber(
            "/aruco_pose",
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )

    def pose_callback(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.last_pose_time = time.time()

    def send_command(self, command):

        rospy.loginfo("Sending command: %s", command)

        self.command_pub.publish(String(data=command))

        # Allow the ROS message to be processed.
        rospy.sleep(0.3)

    def wait_for_controller(self):

        rospy.loginfo("Waiting for end-effector controller...")

        while not rospy.is_shutdown():

            if self.command_pub.get_num_connections() > 0:
                rospy.loginfo("End-effector controller connected.")
                return True

            rospy.sleep(0.2)

        return False

    def run(self):

        if not self.wait_for_controller():
            return

        # ==========================================
        # STEP 1: Wait for system initialization
        # ==========================================

        rospy.loginfo("Waiting for Arduino initialization...")
        rospy.sleep(2.0)

        # ==========================================
        # STEP 2: Activate electromagnet
        # ==========================================

        rospy.loginfo("Activating electromagnet at startup...")

        # Send the command several times to ensure reception
        for i in range(3):
            self.send_command('m')
            rospy.sleep(0.5)

        rospy.loginfo("Initial electromagnet activation commands sent.")

        # ==========================================
        # STEP 3: Wait for ArUco alignment
        # ==========================================

        rospy.loginfo("Waiting for ArUco alignment...")

        while not rospy.is_shutdown():

            pose_is_fresh = (
                self.x is not None
                and time.time() - self.last_pose_time < 1.0
            )

            pose_is_aligned = (
                pose_is_fresh
                and abs(self.x) < 0.01
                and abs(self.y) < 0.01
                and self.z < 0.12
            )

            if pose_is_aligned:

                rospy.loginfo(
                    "ArUco conditions satisfied: "
                    "x=%.4f, y=%.4f, z=%.4f",
                    self.x, self.y, self.z
                )

                break

            rospy.sleep(0.05)

        if rospy.is_shutdown():
            return

        # ==========================================
        # STEP 4: Stop caterpillars
        # ==========================================

        rospy.loginfo("Stopping caterpillars...")

        self.send_command('e')

        # ==========================================
        # STEP 5: Ensure electromagnet is ON
        # ==========================================

        rospy.loginfo("Activating electromagnet before extension...")

        self.send_command('m')

        # ==========================================
        # STEP 6: Extend linear actuator
        # ==========================================

        rospy.loginfo("Extending linear actuator...")

        self.send_command('q')

        # ==========================================
        # STEP 7: Wait 60 seconds
        # ==========================================

        rospy.loginfo("Waiting 15 seconds...")

        deadline = time.time() + 15.0

        while not rospy.is_shutdown() and time.time() < deadline:
            rospy.sleep(0.1)

        if rospy.is_shutdown():
            return

        # ==========================================
        # STEP 8: Deactivate electromagnet
        # ==========================================

        rospy.loginfo("Deactivating electromagnet...")

        self.send_command('n')

        # ==========================================
        # STEP 9: Retract linear actuator
        # ==========================================

        rospy.loginfo("Retracting linear actuator...")

        self.send_command('r')

        rospy.loginfo("Deployment sequence completed.")


if __name__ == "__main__":

    rospy.init_node("deploy_sequence")

    sequence = DeploySequence()
    sequence.run()

    rospy.spin()
