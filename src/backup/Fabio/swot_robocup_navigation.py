#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import sys 
import rospy
import os
import math as ma
import time
import random
import numpy as np
import pandas as pd

from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

from swot_comm_navigation.srv import *
from swot_comm_navigation.msg import *

class Navigation():
    def __init__(self):
        #Publisher
        #Pub New Goal
        self.pub_goal = rospy.Publisher ('/move_base_simple/goal', PoseStamped, queue_size=1, latch =True)
        #Publish to cmd_vel topic (velocities)
        self.pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1, latch=True)
        #Pub to Cancel current goal
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        #Pub to WS_Aligner Topic
        self.pub_aligner = rospy.Publisher('/ws_align', Bool, queue_size=1, latch=True)

        self.pub_tape_detected = rospy.Publisher('/barrier_detected', Bool, queue_size=1, latch=True)

        #Subscriber
        #Subscribe to LaserScan topic
        self.sub_laser = rospy.Subscriber('/scan_front', LaserScan, self.getLaser)
        #Subscribe to WS_Aligner
        self.sub_aligner = rospy.Subscriber('/ws_align', Bool, self.get_ws_align)
        #Subscribe to /move_base/result and get the message, when the robot reaches his goal pose
        self.reachgoal = rospy.Subscriber('/move_base/result', MoveBaseActionResult,self.getres)

        self.tape_detected = rospy.Subscriber('/barrier_detected', Bool, self.getTape)

        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.getPose)
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

        #Services
        #Service to get the next Task (WS)
        self.service = rospy.Service('/swot_next_destination', Swot_Comm_Navigation, self.getWS)

        #Variables
        self.arrived = False
        self.aligned = False
        self.calculate_distance = False
        self.all_path_lenghts = []
        self.shift_count = 0
        self.min_angle_current = 0
        self.min_angle_current_arr = []
        self.count = 0
        self.recovery = False
        self.recovery_counter = 0
        self.ws_driver = False
        self.status =""
        self.old_ws = ""
        self.tape_counter = 0

        self.filepath = os.path.dirname(os.path.realpath(__file__)) + '/WS_Positions.csv'

        rospy.sleep(1)
        rospy.loginfo("Start the navigation")
        rospy.loginfo("Ready to recieve the first goal pose")

        #Randomly send the Service Areas out of the ws_list[] 
        #self.sendWSrandom()

    def sendWSrandom(self):
        while True:
            # Fill in the list with the desired workspaces
            ws_list=["TT01", "SH01", "WS01", "WS02","WS03", "WS04", "WS05", "WS06"]
            next_ws = random.choice(ws_list)
            #Check if old_ws is same as next_ws, to prevent, that the next destination is the same workspace as before
            while self.old_ws == next_ws:
                next_ws = random.choice(ws_list)
            self.old_ws = next_ws
            #Call the Service
            WS_rand = rospy.ServiceProxy('/swot_next_destination', Swot_Comm_Navigation)
            ws_state = WS_rand(next_ws)
            while ws_state.finish==False:
                ws_state = WS_rand(next_ws)
            rospy.loginfo("Navigation Task finished")

    def getWS(self, ws_message): # This function gets called, when a request is sent to the Service
        self.workspace_number = ws_message.destination
        if self.workspace_number != "SHIFT" and self.workspace_number != "START":
            rospy.loginfo("Got new goal, the next destination is: " + str(self.workspace_number)) 
            self.sendgoal()
        elif self.workspace_number == "SHIFT":
            rospy.loginfo("Shift the robot along the Workspace")
            self.shift()
        elif self.workspace_number == "START":
            rospy.loginfo("Drive to the START pose")
            self.sendgoal()
        while self.aligned == False:
            self.sub_aligner = rospy.Subscriber('/ws_align',Bool)
        if self.aligned == True:
            self.aligned = False
            if self.status == "FINISHED":
                rospy.loginfo("Navigation Task finished!")
                self.calculate_distance = True
                return Swot_Comm_NavigationResponse("FINISHED")
            elif self.status == "FAILED":
                rospy.logerr("Navigation Task failed")
                return Swot_Comm_NavigationResponse("FAILED")
            elif self.status == "TAPE":
                self.calculate_distance = True
                return Swot_Comm_NavigationResponse("TAPE")

    def get_ws_align(self, align_data):
        # When the goal is reached, set this to true in order to process the rotation to align to the workspace
        if align_data.data == True:
            self.aligned=True
        else:
            return
            
    def makePlan(self): 
        rospy.wait_for_service('move_base/make_plan')
        pd_data = pd.read_csv(self.filepath, header=0)
        start_col = pd_data.columns.get_loc(self.workspace_number)
        start_pose = pd_data.iloc[:,start_col]
        all_path_lengths = []
        makePlan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
        Start = PoseStamped()
        Start.header.seq = 0
        Start.header.stamp = rospy.Time.now()
        Start.header.frame_id = "map"
        if self.status == "TAPE":
            Start.pose.position.x = self.x
            Start.pose.position.y = self.y
            Start.pose.position.z = 0
            Start.pose.orientation.x = 0
            Start.pose.orientation.y = 0
            Start.pose.orientation.z = self.z
            Start.pose.orientation.w = self.w
            #Set start col to a huge value, so when we make plans, we do it from our current location and not the previous aimed Service Area!
            start_col = 20
        else:
            Start.pose.position.x = start_pose[0]
            Start.pose.position.y = start_pose[1]
            Start.pose.position.z = 0
            Start.pose.orientation.x = 0
            Start.pose.orientation.y = 0
            Start.pose.orientation.z = start_pose[5]
            Start.pose.orientation.w = start_pose[6]

        for goal_col in range (1,9):
            if (goal_col != start_col):
                goal_pose = pd_data.iloc[:,goal_col]
                goal_ws = pd_data.columns[goal_col]
                Goal = PoseStamped()
                Goal.header.seq = 0
                Goal.header.stamp = rospy.Time.now()
                Goal.header.frame_id = "map"
                Goal.pose.position.x = goal_pose[0]
                Goal.pose.position.y = goal_pose[1]
                Goal.pose.position.z = 0
                Goal.pose.orientation.x = 0
                Goal.pose.orientation.y = 0
                Goal.pose.orientation.z = goal_pose[5]
                Goal.pose.orientation.w = goal_pose[6]

                req = GetPlan()
                req.start = Start
                req.goal = Goal
                req.tolerance = 0.5
                pathlength = 0
                resp = makePlan(req.start, req.goal, req.tolerance)
                for i in range(len(resp.plan.poses)-2):
                    x1 = resp.plan.poses[i].pose.position.x
                    y1 = resp.plan.poses[i].pose.position.y
                    x2 = resp.plan.poses[i+1].pose.position.x
                    y2 = resp.plan.poses[i+1].pose.position.y
                    two_point_distance = ma.sqrt(ma.pow((x2 - x1),2)+ ma.pow((y2 - y1),2))
                    pathlength +=two_point_distance
                all_path_lengths.append(pathlength)

            elif (goal_col == start_col):
                all_path_lengths.append(1000)
    
        rospy.set_param('WS_Distances', {'WS01': all_path_lengths[0],
        'WS02': all_path_lengths[2],
        'WS03': all_path_lengths[2],
        'WS04': all_path_lengths[3], 
        'WS05': all_path_lengths[4],
        'WS06': all_path_lengths[5],
        'TT': all_path_lengths[6], 
        'SH': all_path_lengths[7]})

        self.calculate_distance = False
        if self.status == "TAPE":
            self.pub_aligner.publish(True)

    def sendgoal(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        pd_data = pd.read_csv(self.filepath, header=0)
        col = pd_data.columns.get_loc(self.workspace_number)
        #rospy.loginfo("The next Workspace is: " + str(self.workspace_number))
        
        goal_pose = pd_data.iloc[:,col]
        goal.pose.position.x = goal_pose[0]
        goal.pose.position.y = goal_pose[1]
        goal.pose.position.z = goal_pose[2]
        goal.pose.orientation.x = goal_pose[3]
        goal.pose.orientation.y = goal_pose[4]
        goal.pose.orientation.z = goal_pose[5]
        goal.pose.orientation.w = goal_pose[6]

        rospy.sleep(0.1)
        self.pub_goal.publish(goal)

    def getres(self, data):
        self.goal_status = data.status.status
        while(data.status.status !=3):
            if (data.status.status == 4):
                rospy.logwarn("SWOT-Bot can't reach the desired Service Area")
                self.startRecovery()
                return
        if (data.status.status == 3):
            if (self.recovery == False):
                if self.workspace_number != "START":
                    self.arrived = True
                    rospy.loginfo("Goal Reached! Now drive to the Service Area as close as possible")
                else:
                    rospy.loginfo("START pose reached!")
                    self.status = "FINISHED"
                    self.pub_aligner.publish(True)
            elif (self.recovery == True):
                self.sendgoal()
                self.recovery = False
    
    def startRecovery(self):
        #Cancel the current goal pose
        start_time_recovery = time.time()
        duration = rospy.get_param('/swot_robocup_navigation/duration_recovery')
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time_recovery
            if elapsed_time > duration:
                self.status = "FAILED"
                rospy.logwarn("Recovery takes too much time, Abort Recovery")
                cancel_msg = GoalID()
                cancel_msg.stamp = rospy.Time.now()
                self.pub_cancel.publish(cancel_msg)
                self.pub_aligner.publish(True)
                return

            else:
                self.recovery = True
                cancel_msg = GoalID()
                cancel_msg.stamp = rospy.Time.now()
                self.pub_cancel.publish(cancel_msg)
                rospy.loginfo("Start additional Recovery Behaviour")
                rospy.loginfo("At first, perform an in place rotation to clear out the costmaps")
                vel_msg = Twist()
                sec = time.time()
                sec2 = sec+4
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0.2
                while (1): 
                    if sec2<sec:
                        break
                    self.pub_vel.publish(vel_msg)
                    sec = time.time()
                vel_msg.linear.z = 0
                self.pub_vel.publish(vel_msg)
                rospy.sleep(1)

                goal = PoseStamped()
                goal.header.seq = 1
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = 'map'

                if (self.recovery_counter < 1):
                    rospy.loginfo("Drive back to the START Position")
                    goal.pose.position.x = 2.282
                    goal.pose.position.y = -2.073
                    goal.pose.position.z = 0.0
                    goal.pose.orientation.x = 0.0
                    goal.pose.orientation.y = 0.0
                    goal.pose.orientation.z = 0.959
                    goal.pose.orientation.w = 0.285
                    self.recovery_counter += 1

                else:
                    rospy.loginfo("Drive to an alternative position, because the START position is not reachable at the moment")
                    goal.pose.position.x = 2.244
                    goal.pose.position.y = -2.085
                    goal.pose.position.z = 0.0
                    goal.pose.orientation.x = 0.0
                    goal.pose.orientation.y = 0.0
                    goal.pose.orientation.z = -0.291
                    goal.pose.orientation.w = 0.957

                    self.recovery_counter = 0

                rospy.sleep(0.1)
                self.pub_goal.publish(goal)

    def getLaser(self, data):
        if self.arrived == False:
            if self.calculate_distance == True and self.status !="TAPE":
                self.makePlan()
            #if we put the wait here we are stuck with the old data at this point and this old message will be processed
            # as soon as the condition is met leading to unwanted behaviour
            return
        else:
            vel_msg = Twist()
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.
            self.ranges = list(data.ranges)
            while min(self.ranges) == 0.0020000000949949026:
                self.ranges.remove(0.0020000000949949026)
            self.min_distance = min(self.ranges)
            if self.min_distance > 0.15:
                self.pub_vel.publish(vel_msg)
            else:
                vel_msg.linear.x = 0
                self.pub_vel.publish(vel_msg)
                rospy.loginfo("Robot is as close as possible.")
                self.status = "FINISHED"
                self.arrived = False
                self.pub_aligner.publish(True)
                return

    def shift(self):
        rospy.loginfo("SHIFT Robot")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        sec = time.time()
        sec2 = sec+3.5
        if self.shift_count == 0:
            #Shift to the left
            vel_msg.linear.y = 0.1
            self.shift_count+=1
        elif self.shift_count > 0:
            #Shift to the right
            vel_msg.linear.y = -0.1
            self.shift_count = 0
        while (1): 
            if sec2<sec:
                break
            self.pub_vel.publish(vel_msg)
            sec = time.time()
        vel_msg.linear.y = 0
        self.pub_vel.publish(vel_msg)
        rospy.loginfo("Reached End Position, Ready for SCAN")
        self.pub_aligner.publish(True)
        return

    def getTape(self, tape_data):
        if self.tape_counter == 0:
            if tape_data.data == True:
                cancel_msg = GoalID()
                cancel_msg.stamp = rospy.Time.now()
                self.pub_cancel.publish(cancel_msg)
                self.tape_counter = 1
                self.status = "TAPE"
                self.pub_tape_detected.publish(False)
                rospy.sleep(1)
                self.makePlan()

    def getPose(self, pose_data):
        self.x = pose_data.pose.pose.position.x
        self.y = pose_data.pose.pose.position.y
        self.z = pose_data.pose.pose.orientation.z
        self.w = pose_data.pose.pose.orientation.w
    
if __name__=='__main__':
    rospy.init_node('Swot_Navigation_Node', anonymous=True)
    time.sleep(1)
    newgoal = Navigation()
    rospy.spin()
