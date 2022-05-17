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
import sys
import pickle as pkl
import csv


from geometry_msgs.msg import PoseStamped, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import Empty
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from navigation.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

class multipoint():
    def __init__(self):
        self.node_name = rospy.get_name()

        # initiallize boat status
        self.auto = 0
        self.epoch = 0
        self.count = 0
        self.iteration = 10
        self.record = np.zeros([self.iteration])
        self.record_collision = np.zeros([self.iteration])
        self.record_time = np.zeros([self.iteration])
        self.collision = 0
        self.collision_states = False
        self.start_time = 0

        self.robot_pose = [0.0, 0.0]
        self.stop = False
        self.cmd_drive = Twist()
        self.robot_radius = float(rospy.get_param("~robot_radius", "0.5"))
        self.yaw = 0
        self.model = rospy.get_param("~model", "robot")
        print("Robot Radius: ", self.robot_radius)
        
        # For EE6F world waypoint navigation
        # (EE632, EE622, EE623, elevator, EE616, EE613, EE605)
        self.pt_list = [(21, -23.5, -3.13),(12, -23.5, -3.13),(-6, -23.5, -3.13),(-12, -18.5, 1.57),(-12, 6.5, 1.57),(-20, 27.5, -3.13),(-30.5, 27.5, -3.13)]
        
        self.final_goal = (-30.5, 27.5, -3.13) # The final goal that you want to arrive
        self.goal = None
        # self.p_list = []

        # self.p_list = self.pt_list
        # print(self.p_list)

        # for i,point in enumerate(self.p_list):
        #     self.points.put(point)

        # self.goal = self.points.get()
        # print("---", type(self.points), "---")
        # print ("boat: ", self.goal)

        if self.start_time == 0:
            time.sleep(2.0)
            self.start_time = rospy.Time.now()
            print(self.start_time)

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.Markers = MarkerArray()
        self.Markers.markers = []
        
        self.sub = rospy.Subscriber("odometry", Odometry, self.cb_odom, queue_size=1)
        self.sub_collision = rospy.Subscriber("bumper_states", ContactsState, self.cb_collision, queue_size=1)

        self.initial_state()

        

    def cb_odom(self, msg):
        self.get_marker()
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        goal_distance = self.get_distance(self.robot_pose, self.goal)
        
        if self.final_goal == self.goal and goal_distance < self.robot_radius and self.stop == False:
            self.record[self.epoch] = 1
            self.record_collision[self.epoch] = str(self.collision)
            time = rospy.Time.now()
            print((time - self.start_time)/1000000000)
            self.record_time[self.epoch] = str((time - self.start_time)/1000000000)
            print (self.epoch, self.record[self.epoch], self.record_collision[self.epoch])
            self.epoch += 1
            self.initial_state()
            self.start_time = rospy.Time.now()
            print(self.start_time)
        elif goal_distance < self.robot_radius and self.stop == False:
            self.points.put(self.goal)
            self.goal = self.points.get()
            print ("boat: ", self.goal)

        if self.epoch == self.iteration and self.stop == False:
            folder = '/home/kiat_thesis/master_thesis/bag'
            record_name = 'uwb_navigation.csv'
            fileObject = open(folder + "/" + record_name, 'w')
            writer = csv.writer(fileObject)
            writer.writerow(self.record)
            writer.writerow(self.record_collision)
            writer.writerow(self.record_time)
            fileObject.close()
            self.stop = True

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        self.publish_goal(self.goal[0], self.goal[1], self.goal[2])
        self.pub_points.publish(self.Markers)

    def cb_collision(self, msg):
        if self.collision_states == True:
            if msg.states == [] and self.count > 1000:
                self.collision_states = False
                # print(self.collision_states)
            else:
                self.count += 1
                # print(self.count)
        elif msg.states != [] and self.count == 0:
            self.collision_states = True
            self.collision += 1
            print(self.collision)
            print(self.collision_states)
        else:
            self.collision_states = False
            self.count = 0


    def publish_goal(self, pose_x, pose_y, yaw):
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal_pos = PoseStamped()
        goal_pos.pose.position.x = float(pose_x)
        goal_pos.pose.position.y = float(pose_y)
        goal_pos.pose.position.z = 0.0

        goal_pos.pose.orientation.x = quat[0]
        goal_pos.pose.orientation.y = quat[1]
        goal_pos.pose.orientation.z = quat[2]
        goal_pos.pose.orientation.w = quat[3]
        goal_pos.header.stamp = rospy.Time.now() 
        goal_pos.header.frame_id = "map" 
        # rospy.loginfo(goal_pos)
        self.pub_goal.publish(goal_pos)

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def initial_state(self):
        # start position
        yaw = -3.13
        self.state_msg = ModelState()
        self.state_msg.model_name = self.model
        
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        self.state_msg.pose.orientation.x = quat[0]
        self.state_msg.pose.orientation.y = quat[1]
        self.state_msg.pose.orientation.z = quat[2]
        self.state_msg.pose.orientation.w = quat[3]

        self.state_msg.pose.position.x = 36.5
        self.state_msg.pose.position.y = -23.5
        self.state_msg.pose.position.z = 0.13

        if self.stop == False:
            # initial goal pose
            self.points = queue.Queue(maxsize=20)
            self.p_list = []
            self.p_list = self.pt_list
            print(self.p_list)

            for i,point in enumerate(self.p_list):
                self.points.put(point)

            self.goal = self.points.get()
            print("---", type(self.points), "---")
            print ("boat: ", self.goal)

            self.collision = 0

            self.reset_model(self.state_msg)

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