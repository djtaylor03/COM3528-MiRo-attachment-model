#!/usr/bin/env python3

import os
from math import radians  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import random
import time

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped, Pose2D # ROS cmd_vel (velocity control) message


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2

class Play():
    """
    Script settings below
    """
    TICK = 0.02  # This is the update interval for the main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.4  # Linear speed when kicking the ball (m/s)
    DEBUG = False  # Set to True to enable debug views of the cameras
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    


    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """

        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)


    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        self.NODE_EXISTS = True
        rospy.init_node("play", anonymous=True)
        # Give it some time to make sure everything is initialised
        
        # Initialise CV Bridge
        #self.image_converter = CvBridge()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
       
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        
        # secure av=0 am = 0
        # avoidant av > 0 am = 0  av= 20 is good value dont go too high
        # ambivalent av = 0 am >0 am= 0.9 is max

        # if av = 0:
        #     if am = 0:
        #         self.attachment = "secure"
        #     else:
        #         self.attachment = "ambivalent"
        # else:
        #     self.attachment = "avoidant"

        self.attachment = "secure" 
        #self.attachment = "avoidant"
        #self.attachment = "ambivalent"

        

        rospy.Subscriber(
            # the topic that gives the robot's pose in a 2D space
            topic_base_name + "/sensors/body_pose",
            Pose2D,
            self.body_pose,
        )

        rospy.sleep(2.0)

        self.x0 = self.x

    def body_pose(self, data):
        self.x = data.x


    def play(self):
        if self.rand == 0 or self.rand == 1:
            if self.attachment == "secure":
                self.drive(self.FAST, self.FAST)
            else:
                self.drive(self.SLOW, self.SLOW)
        elif self.rand == 2: 
            if self.attachment == "secure":
                self.drive(-self.FAST, self.FAST)
            elif self.attachment == "avoidant":
                self.drive(self.SLOW, self.FAST)
            else:
                self.drive(-self.SLOW, self.SLOW)
        else:
            if self.attachment == "secure":
                self.drive(self.FAST, -self.FAST)
            elif self.attachment == "avoidant":
                self.drive(self.FAST, self.SLOW)
            else:
                self.drive(self.SLOW, -self.SLOW)
        if self.counter > self.timer:
            print ("switch", self.rand)
            self.rand = random.randint(0, 3)
            self.timer = random.randint(1, 100)
            self.counter = 0


    def loop(self):
        """
        Main control loop
        """
        print("Miro plays")
        # Main control loop iteration counter
        self.counter = 0
        self.rand = 0
        self.timer = 50
        # This switch loops through MiRo behaviours:
        # Find ball, lock on to the ball and kick ball
        self.status_code = 0
        while not rospy.core.is_shutdown():

            

            self.play()
            print (self.x0, self.x)


            self.counter += 1
            rospy.sleep(self.TICK)


if __name__ == "__main__":
    play = Play()  # Instantiate class
    play.loop()  # Run the main control loop
