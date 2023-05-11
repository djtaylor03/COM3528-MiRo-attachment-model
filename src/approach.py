#!/usr/bin/env python3
# imports
from codecs import ignore_errors
import os
from unittest import skip
import numpy as np

import rospy
from geometry_msgs.msg import TwistStamped, Pose2D

# TODO Take in odometry
# TODO Feed in approach coordinates
# TODO turn towards the coordinates (need to know where MiRo is facing)
# TODO move until there

class ApproachClient:

    def __init__(self) -> None:

        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
        rospy.init_node("approach", anonymous=True)

        # PUBLISHERS
        self.pub_cmd_vel = rospy.Publisher(
            topic_root + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # SUBSCRIBERS
        rospy.Subscriber(
            # the topic that gives the robot's pose in a 2D space
            topic_root + "/sensors/body_pose",
            Pose2D,
            self.body_pose,
        )

    def body_pose(self, data):
        print(data)

    def loop(self):
        while not rospy.core.is_shutdown():
            self.body_pose



if __name__ == "__main__":
    main = ApproachClient() # Instantiate class
    main.loop()             # Begin main loop    