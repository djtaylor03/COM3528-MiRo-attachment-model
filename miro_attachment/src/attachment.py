#!/usr/bin/env python3

import os
from math import radians
import numpy as np
import random


import rospy
from geometry_msgs.msg import TwistStamped, Pose2D
from std_msgs.msg import UInt16

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
        rospy.Subscriber(
            topic_base_name + "/sensors/body_pose", Pose2D, self.body_pose
        )

        #-----------------------------------------#
        #publishers
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        #-----------------------------------------#

        #initiation related to all
        

        #-----------------------------------------#
        #care related init

        # Move the head to default pose
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
        self.epsilonAm = 0.5 # range [-0.1, 0.9]

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
        self.theta0 = self.theta
        self.x0 = self.x
        self.y0 = self.y

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
    def body_pose(self, data):
        self.theta = data.theta
        self.x = data.x
        self.y = data.y

    #PUT APPROACH HERE############
    def approach(self):
        print ("approaching")

        
#-----------------------------------------#
    #explore related methods
    def explore(self):
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
        


        ###play
        # Main control loop iteration counter

        #explore loop counters
        self.counter = 0
        self.rand = 0
        self.timer = 50

        # This switch loops through MiRo behaviours:
        # Find ball, lock on to the ball and kick ball
        self.status_code = 0

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
                print("explore     edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.edist +=0.01
                self.pdist +=0.01
                # self.drive(0.4,0.4)

                ###play
                self.explore()


                self.counter += 1

            else:
                print("approach   edist= "+str(np.round(self.edist,2))+"   pdist= "+str(np.round(self.pdist,2))+"   Robot Need= "+ str(np.round(self.prevX[2],2)))

                self.pdist -= 0.01
                # self.drive(0,0)
                # self.drive(0.4,0.4)
                # print(self.need)

                ###approach
                #self.approach()

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