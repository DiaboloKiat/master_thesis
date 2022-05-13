#! /usr/bin/env python

import numpy as np
import roslib
import rospy
import math
import os
import tf
# import queue #python3
import Queue as queue
import cv2
import struct
import time

from geometry_msgs.msg import PoseStamped, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from navigation.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse

class multipoint():
    def __init__(self):
        self.node_name = rospy.get_name()

        # initiallize boat status
        self.auto = 0
        self.start_station_keeping = False
        self.start_navigation = False

        self.robot_pose = [0.0, 0.0]
        self.stop_pose = []
        self.cmd_drive = Twist()
        self.robot_radius = float(rospy.get_param("~robot_radius", "5.5"))
        self.yaw = 0
        print(self.robot_radius)

        self.points = queue.Queue(maxsize=20)
        self.reverse = False
        
        For wamv world waypoint navigation
        self.pt_list = [(2.5, 2.5),(2.5, -2.5),(-2.5, -2.5),(-2.5, 2.5)]
        
        # # For EE6F world waypoint navigation
        # self.pt_list = [(34, -23.5),(17, -23.5),(-2, -23.5),(-12.09, -23.5),(-12.09, -6.5),(-12.09, 14.5),(-12.09, 27.5),(-30.34, 27.5)]
        self.final_goal = None # The final goal that you want to arrive
        self.goal = self.final_goal
        self.p_list = []

        self.dis4constV = 5.0               # Distance for constant velocity
        self.pos_ctrl_max = 5
        self.pos_ctrl_min = 0.0
        self.pos_station_max = 2
        self.pos_station_min = 0
        self.station_keeping_distance = 0.5      # meters


        if self.reverse :
            while len(self.pt_list) != 0:
                self.p_list.append(self.pt_list.pop())
        else :
            self.p_list = self.pt_list
            print(self.p_list)

        for i,point in enumerate(self.p_list):
            self.points.put(point)

        self.goal = self.points.get()
        print("---", type(self.points), "---")
        print ("boat: ", self.goal)
        
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.Markers = MarkerArray()
        self.Markers.markers = []
        
        self.sub = rospy.Subscriber("odometry", Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        self.get_marker()
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        goal_distance = self.get_distance(self.robot_pose, self.goal)
        
        if self.final_goal != None:
            self.goal = self.final_goal
            self.final_goal = None
            print ("boat: ", self.goal)
        elif goal_distance<self.robot_radius and self.start_station_keeping == False:
            self.points.put(self.goal)
            self.goal = self.points.get()
            print ("boat: ", self.goal)
            if self.goal == (-30.34, 27.5):
                self.start_station_keeping = True

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        self.publish_goal(self.goal[0], self.goal[1])
        self.pub_points.publish(self.Markers)


    def publish_goal(self, pose_x, pose_y):
        robot_pos = PoseStamped()
        robot_pos.pose.position.x = float(pose_x)
        robot_pos.pose.position.y = float(pose_y)
        robot_pos.pose.position.z = 0.0

        robot_pos.pose.orientation.x = 0.0
        robot_pos.pose.orientation.y = 0.0
        robot_pos.pose.orientation.z = 0.0
        robot_pos.pose.orientation.w = 0.0
        robot_pos.header.stamp = rospy.Time.now() 
        robot_pos.header.frame_id = "map" 
        # rospy.loginfo(robot_pos)
        self.pub_goal.publish(robot_pos)

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_marker(self):
        i = 1
        for pt in self.pt_list:
            self.marker = Marker()
            self.marker.header.stamp = rospy.Time.now()
            self.marker.header.frame_id = 'map'
            self.marker.type = self.marker.SPHERE
            self.marker.action = self.marker.ADD
            self.marker.pose.orientation.w = 1
            self.marker.pose.position.x = pt[0]
            self.marker.pose.position.y = pt[1]
            self.marker.id = i
            i = i+1
            self.marker.scale.x = 0.25
            self.marker.scale.y = 0.25
            self.marker.scale.z = 0.25
            self.marker.color.a = 1.0
            self.marker.color.r = 0
            self.marker.color.g = 0
            self.marker.color.b = 1
            self.Markers.markers.append(self.marker)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('multipoint_node',anonymous=False)
    multipoint_node = multipoint()
    rospy.on_shutdown(multipoint_node.on_shutdown)
    rospy.spin()