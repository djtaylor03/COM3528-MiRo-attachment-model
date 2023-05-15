#!/usr/bin/env python3

from lib2to3.pgen2.literals import test
import os
from math import factorial, radians, atan2
import numpy as np
import random
from miro2.lib.types import Pose


import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt16
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import miro2
from miro2.lib import wheel_speed2cmd_vel

class MiRo_attachment:
    TICK = 0.005
    SLOW = 0.1
    FAST = 0.4
    NODE_EXISTS = False

    def __init__(self):
        rospy.init_node("attachment_model", anonymous=True)

        #rospy.sleep(2.0) #maybe remove

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        #-----------------------------------------#
        #subscribers
        #head sensors
        rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_head,
        )

        #body sensors
        rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_body,
        )

        #body pose
        # rospy.Subscriber(
        #     topic_base_name + "/sensors/body_pose", Pose2D, self.body_pose
        # )

        #body pose (to replace?)
        rospy.Subscriber(
            topic_base_name + "/sensors/odom", Odometry, self.body_pose_2
        )

        #-----------------------------------------#
        #publishers
        #movement
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        #-----------------------------------------#

        #initiation related to all
        

        #-----------------------------------------#
        #care related init

        self.touch_data = [None, None] # head and body
        self.edist = 0
        self.pdist = 0
        self.prevX = [-1.0,1.0,-1.0,1.0]

        #--was out
        self.A_approach = lambda x: np.heaviside(x, 0)

        self.h = 0.005

        # secure av=0 am = 0
        # avoidant av > 0 am = 0  av= 20 is good value dont go too high
        # ambivalent av = 0 am >0 am= 0.9 is max
        # higher values will make trait more apparent

        self.epsilonAv = 0.0 # range [-0.1, \infty] 
        self.epsilonAm = 0.0 # range [-0.1, 0.9]

        self.b = 0.5
        self.e = 1.0
        self.dp = 0.0
        self.de = 0.0
        self.k0 = 0.8
        self.delta = 0.1

        #-----------------------------------------#
        #approach related init

        rospy.sleep(2.0)
        #init initial pose
        #self.theta0 = self.theta
        self.x0 = self.x = 0
        self.y0 = self.y = 0
        self.theta0 = self.theta = 0

        self.facing_origin = False

        #-----------------------------------------#
        #explore related init

        if self.epsilonAv == 0:
            if self.epsilonAm == 0:
                self.attachment = "secure"
            else:
                self.attachment = "ambivalent"
        else:
            self.attachment = "avoidant"

        print (self.attachment)

##############################################################################

    #methods related to all

    def drive(self, speed_l = 0.1,speed_r= 0.1):
        msg_cmd_vel = TwistStamped()

        wheel_speed = [speed_l,speed_r]

        (dr,dtheta) = wheel_speed2cmd_vel(wheel_speed)

        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        self.vel_pub.publish(msg_cmd_vel)

#-----------------------------------------#
    #care related methods
    def callback(self, touch_data, index):
        # the touch data is encoded as a bit array
        # formatted strings can handle this natively
        bit_str = "{0:014b}".format(touch_data.data)
        self.touch_data[index] = list(map(int, [*(bit_str)]))

        # Another way is to use bitwise shift
        #[touch_data.data & (1 << i) for i in range(14)]

    def callback_head(self, touch_data):
        self.callback(touch_data, 0)

    def callback_body(self, touch_data):
        self.callback(touch_data, 1)

    def f(self, t, r,ed,pd):
        x1 = r[0]   # parent need
        y1 = r[1]   # parent accumulated need
        x2 = r[2]   # child need
        y2 = r[3]   # child accumulated need
        # pd = 2  # distance based on ambivalence, physical distance, will be inferred later on from sensory data based on their sensitivity
                    # distance based on avoidance, emotional distance, will be inferred later on from sensory data based on their sensitivity
        dx2 = -(2.0 * x2**3 - x2) - y2
        dy2 = self.e * ((self.b * x2) - (self.epsilonAm * pd)) - self.k0 * (self.epsilonAv * (ed - self.delta))
        dx1 = -(2.0 * x1**3 - x1) - y1
        dy1 = self.e * ((self.b * x1) + (self.epsilonAm * pd)) - self.k0 * (self.epsilonAv * (ed - self.delta))
        return np.array([dx1, dy1, dx2, dy2])
    

#-----------------------------------------#
    #approach related methods
    # def body_pose(self, data):
    #     self.theta = data.theta
    #     self.x = data.x
    #     self.y = data.y

    def body_pose_2(self, data):
        #self.theta = data.orientation.theta
        # self.x = data.position.x
        # self.y = data.position.y

        orientation = data.pose.pose.orientation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')


    #PUT APPROACH HERE############
    def approach(self):

        print ("approaching")

        inc_x = self.x0-self.x
        inc_y = self.y0-self.y

        angle_to_origin = atan2(inc_y, inc_x)
        if (abs(angle_to_origin - self.theta) > 0.1) and not self.facing_origin:
            self.drive(self.SLOW, 0)
        else:
            self.facing_origin = True
            self.drive(self.FAST, self.FAST)

        
        rospy.sleep(0.2)
        print (angle_to_origin - self.theta)
        
#-----------------------------------------#
    #explore related methods
    def explore(self):
        self.facing_origin = False
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
            self.rand = random.randint(0, 3)
            self.timer = random.randint(1, 100)
            self.counter = 0


#-----------------------------------------#
    #loop
    def loop(self):
        
        # Main control loop

        #explore loop counters
        self.counter = 0
        self.rand = 0
        self.timer = 50

        self.status_code = 0

        #print (test1)

        while not rospy.core.is_shutdown():
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

            k1 = self.f(1, self.prevX,self.edist,self.pdist)
            k2 = self.f(1 + self.h/2.0, self.prevX + self.h*k1/2.0,self.edist,self.pdist)
            k3 = self.f(1 + self.h/2.0, self.prevX + self.h*k2/2.0,self.edist,self.pdist)
            k4 = self.f(1+ self.h, self.prevX + self.h*k3,self.edist,self.pdist)
            self.prevX = self.prevX + self.h*(k1 + 2*k2 + 2*k3 + k4)/6.0
            if (self.A_approach(self.prevX[2]) == 0):
                # Explore
                #print("explore     edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.edist +=0.01
                self.pdist +=0.01
                # self.drive(0.4,0.4)

                self.explore()
                #print (self.x, self.y, self.theta)


                self.counter += 1

            else:
                # Approach
                #print("approach   edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.pdist -= 0.01
                # self.drive(0,0)
                # self.drive(0.4,0.4)
                # print(self.need)

                self.approach()

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

#-----------------------------------------#
if __name__ == "__main__":
    attachment_model = MiRo_attachment() #Instantiate class
    attachment_model.loop()