#!/usr/bin/env python3
"""
This script makes MiRo look for a blue ball and kick it

The code was tested for Python 2 and 3
For Python 2 you might need to change the shebang line to
#!/usr/bin/env python
"""
# Imports
##########################
import math
import os
from math import radians  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
from std_msgs.msg import UInt16


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient:
    """
    Script settings below
    """
    TICK = 0.005  # This is the update interval for the main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.4  # Linear speed when kicking the ball (m/s)
    DEBUG = True  # Set to True to enable debug views of the cameras
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node


 
    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("kick_blue_ball", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Initialise CV Bridge
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Create two new subscribers to receive camera images with attached callbacks

        rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_head,
        )
        rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_body,
        )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        # Create handle to store images
        self.input_camera = [None, None]
        # New frame notification
        self.new_frame = [False, False]
        # Create variable to store a list of ball's x, y, and r values for each camera
        self.ball = [None, None]
        # Set the default frame width (gets updated on receiving an image)
        self.frame_width = 640
        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0


        # Move the head to default pose
        self.touch_data = [None, None] # head and body
        self.edist = 0
        self.pdist = 0
        self.prevX = [-1.0,1.0,-1.0,1.0]

    def drive(self, speed_l = 0.1,speed_r= 0.1):
        msg_cmd_vel = TwistStamped()

        wheel_speed = [speed_l,speed_r]

        (dr,dtheta) = wheel_speed2cmd_vel(wheel_speed)

        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        self.vel_pub.publish(msg_cmd_vel)

    def callback_head(self, touch_data):
        self.callback(touch_data, 0)

    def callback_body(self, touch_data):
        self.callback(touch_data, 1)
    def callback(self, touch_data, index):
        # the touch data is encoded as a bit array
        # formatted strings can handle this natively
        bit_str = "{0:014b}".format(touch_data.data)
        self.touch_data[index] = list(map(int, [*(bit_str)]))

        # Another way is to use bitwise shift
        #[touch_data.data & (1 << i) for i in range(14)]
    def loop(self):
        """
        Main control loop
        """
        # print("MiRo plays ball, press CTRL+C to halt...")
        # Main control loop iteration counter
        self.counter = 0
        # This switch loops through MiRo behaviours:
        # Find ball, lock on to the ball and kick ball
        self.status_code = 0
        while not rospy.core.is_shutdown():

            # Step 1. Find ball
            # print("Head sensor array: {}".format(main.touch_data[0]))
            # print("Body sensor array: {}".format(main.touch_data[1]))
            # print("-"*61)
            try:
                if 1 in self.touch_data[0]:
                    self.edist -= 0.01*np.sum(self.touch_data)
                if self.edist < -10:
                    self.edist = -10
                if self.edist > 2:
                    self.edist = 2
            except:
                print('none')
        
            
            # print("Emotional Distance: "+ str(np.round(self.need,0)))

            k1 = f(1, self.prevX,self.edist,self.pdist)
            k2 = f(1 + h/2.0, self.prevX + h*k1/2.0,self.edist,self.pdist)
            k3 = f(1 + h/2.0, self.prevX + h*k2/2.0,self.edist,self.pdist)
            k4 = f(1+ h, self.prevX + h*k3,self.edist,self.pdist)
            self.prevX = self.prevX + h*(k1 + 2*k2 + 2*k3 + k4)/6.0
            if (A_approach(self.prevX[2]) == 0):
                print("explore     edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.edist +=0.01
                self.pdist +=0.01
                # self.drive(0.4,0.4)
            else:
                print("approach   edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.pdist -= 0.01
                # self.drive(0,0)
                # self.drive(0.4,0.4)
                # print(self.need)
            try:
                if self.pdist > 5:
                    self.pdist = 5
                if self.pdist < 0.5:
                    self.pdist = 0.5
            except:
                print('none')
            # print(self.pdist)

            # print(self.prevX)
            rospy.sleep(self.TICK)


A_approach = lambda x: np.heaviside(x, 0)

h = 0.005

#
# parameters to change
#
# secure av=0 am = 0
# avoidant av > 0 am = 0  av= 20 is good value dont go too high
# ambivalent av = 0 am >0 am= 0.9 is max
# higher values will make trait more apparent

epsilonAv = 20.0 # range [-0.1, \infty] 
epsilonAm = 0.0 # range [-0.1, 0.9]

#
#
#

b = 0.5
e = 1.0
dp = 0.0
de = 0.0
k0 = 0.8
delta = 0.1

def f(t, r,ed,pd):
        x1 = r[0]   # parent need
        y1 = r[1]   # parent accumulated need
        x2 = r[2]   # child need
        y2 = r[3]   # child accumulated need
        # pd = 2  # distance based on ambivalence, physical distance, will be inferred later on from sensory data based on their sensitivity
                    # distance based on avoidance, emotional distance, will be inferred later on from sensory data based on their sensitivity
        dx2 = -(2.0 * x2**3 - x2) - y2
        dy2 = e * ((b * x2) - (epsilonAm * pd)) - k0 * (epsilonAv * (ed - delta))
        dx1 = -(2.0 * x1**3 - x1) - y1
        dy1 = e * ((b * x1) + (epsilonAm * pd)) - k0 * (epsilonAv * (ed - delta))
        return np.array([dx1, dy1, dx2, dy2])



# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop