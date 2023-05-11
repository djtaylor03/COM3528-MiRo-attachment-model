#!/usr/bin/env python3

import os
from math import radians  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import random
import time

import rospy  # ROS Python interface


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2

class RequestCare():
   
    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        rospy.init_node("request care", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        
    def loop(self):
        """
        Main control loop
        """
        print("Miro Requests Care")
        
        self.give_up = False
        while not rospy.core.is_shutdown() or not self.give_up:
            
            


if __name__ == "__main__":
    approach = Approach()  # Instantiate class
    approach.loop()  # Run the main control loop
