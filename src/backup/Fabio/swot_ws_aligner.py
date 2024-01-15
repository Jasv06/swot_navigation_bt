#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import sys 
import rospy
import os
import math as ma
import time
import numpy as np

import message_filters

from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID


from robocup_tasks.srv import *
from robocup_tasks.msg import *

class WSAligner():
    def __init__(self):
        #Subscriber
        self._data_front = rospy.Subscriber('/scan_front', LaserScan, self.getLaser)

        #Publisher
        self.cancelling = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.finisher = rospy.Publisher('/ws_align', Bool, queue_size=1, latch=True)
        self.pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1, latch=True)


        #Define Init Values for variables and arrays
        self.degree_round = []
        self.min_angle_current = 0
        self.min_angle_current_arr=[]
        self.count = 0



        self.arrived=False
        self.aligned_1 = True
        self.aligned_2 = False
        self.left = False
        self.right = False

        #Subscribe to /move_base/result and get the message, when the robot reaches his goal pose
        self.reachgoal = rospy.Subscriber('/move_base/result', MoveBaseActionResult,self.getres)

        # 270 Degree FOV and 0,33 resolution -> ca. 818 Beams over the 270 Degree FOV
        # 135 Degree means, that the robot is parallel to the Workspace in front of him

        # Upper Border for LaserScan FOV -> 585 means 145 Degree
        self.upl=600
        # Lower Border for LaserScan FOV -> 505 means 125 Degree
        self.lowl=500
      
    def getres(self, data):
        # When the goal is reached, set this to true in order to process the rotation to align to the workspace
        if (data.status.status == 3):
            self.arrived=True
        
        else:
            print("Zielpunkt konnte nicht erreicht werden")

    def getLaser(self, data):
        if self.arrived == False:
            #if we put the wait here we are stuck with the old data at this point and this old message will be processed
            # as soon as the condition is met leading to unwanted behaviour
            return                       
        elif self.aligned_1 == True and self.aligned_2 == False:
                vel_msg = Twist()
                vel_msg.linear.x = 0.1
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0

                self.ranges = list(data.ranges)

                while min(self.ranges) == 0.0020000000949949026:
                    self.ranges.remove(0.0020000000949949026)
                self.min_distance = min(self.ranges)
                if self.min_distance > 0.15:
                    self.pub_vel.publish(vel_msg)

                else:
                    vel_msg.linear.x = 0
                    self.pub_vel.publish(vel_msg)
                    print("Robot is as close as possible.")
                    #self.aligned_1 = False
                    self.aligned_2 = True
                    self.left = True
                    rospy.sleep(5)
  

        elif self.aligned_1 == True and self.aligned_2 ==  True and self.left == True and self.right == False:  
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = -0.1
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            if data.ranges[500] > 0.4: #760 in simulation Bei 15 cm Abstand zu Workspace 20 cm Abstand bei allen Zielpunkten einspeichern
                self.left = False
                self.right = True
                vel_msg.linear.y = 0
                self.pub_vel.publish(vel_msg)
                print ("Right Corner")
                rospy.sleep(5)
                return
            else:  
                self.pub_vel.publish(vel_msg)


        elif self.aligned_1 == True and self.aligned_2 ==  True and self.left == False and self.right == True:  
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0.1
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            sec = time.time()
            sec2 = sec+3.5
            while (1): 
                if sec2<sec:
                    break
                self.pub_vel.publish(vel_msg)
                sec = time.time()
            vel_msg.angular.z = 0
            self.pub_vel.publish(vel_msg)
            self.right = False
            self.aligned_1 = False
            self.aligned_2 = False
            self.arrived = False
            print ("Left Corner")
            rospy.sleep(5)
            return





if __name__=='__main__':
    rospy.init_node('WSAligner', anonymous=True)
    wsalign1 = WSAligner()
    rospy.spin()