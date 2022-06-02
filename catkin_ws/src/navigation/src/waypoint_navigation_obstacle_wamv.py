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
import csv

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
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

from PID import PID_control

class waypoint_navigation_obstacle():
    def __init__(self):
        self.node_name = rospy.get_name()

        # initiallize boat status
        self.auto = 0
        self.epoch = 0
        self.count = 0
        self.iteration = 1
        self.record = np.zeros([self.iteration])
        self.record_collision = np.zeros([self.iteration])
        self.record_time = np.zeros([self.iteration])
        self.collision = 0
        self.collision_states = False
        self.start_station_keeping = False
        self.start_time = 0
        self.goal = None
        self.frame = rospy.get_param("~frame", "map")
        self.docking = False

        self.robot_pose = [0.0, 0.0]
        self.stop = False
        self.cmd_drive = Twist()
        self.robot_radius = float(rospy.get_param("~robot_radius", "0.5"))
        self.yaw = 0
        self.joy = rospy.get_param("~joy", "joy_teleop/joy")
        self.cmd_vel = rospy.get_param("~cmd_vel", "joy_teleop/cmd_vel")
        self.veh = rospy.get_param("~veh", "husky")
        print("Robot Radius: ", self.robot_radius)


        self.dis4constV = 5.0               # Distance for constant velocity
        self.pos_ctrl_max = 5
        self.pos_ctrl_min = 0.0
        self.pos_station_max = 2
        self.pos_station_min = 0
        self.station_keeping_distance = 0.5      # meters
        
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel, Twist, queue_size=1)

        self.sub_joy = rospy.Subscriber(self.joy, Joy, self.cb_joy, queue_size=1)
        self.sub_scan = rospy.Subscriber("RL/scan", LaserScan, self.get_scan, queue_size=1)
        self.sub = rospy.Subscriber("odometry", Odometry, self.cb_odom, queue_size=1)
        self.sub_collision = rospy.Subscriber("bumper_states", ContactsState, self.cb_collision, queue_size=1)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal,queue_size=1)

        self.pos_control = PID_control("Position")
        self.ang_control = PID_control("Angular")

        self.ang_station_control = PID_control("Angular_station")
        self.pos_station_control = PID_control("Position_station")

        self.pos_srv = Server(pos_PIDConfig, self.cb_pos_pid, "Position")
        self.ang_srv = Server(ang_PIDConfig, self.cb_ang_pid, "Angular")
        self.pos_station_srv = Server(pos_PIDConfig, self.cb_pos_station_pid, "Angular_station")
        self.ang_station_srv = Server(ang_PIDConfig, self.cb_ang_station_pid, "Position_station")

        self.initialize_PID()

    def cb_odom(self, msg):
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # print(self.robot_pose)

        if self.auto == 0:
            return

        goal_distance = self.get_distance(self.robot_pose, self.goal)

        quaternion = (msg.pose.pose.orientation.x,\
                        msg.pose.pose.orientation.y,\
                        msg.pose.pose.orientation.z,\
                        msg.pose.pose.orientation.w)

        _, _, self.yaw = tf.transformations.euler_from_quaternion(quaternion)

        goal_angle = self.get_goal_angle(self.yaw, self.robot_pose, self.goal)


        if self.docking == False:
            if self.lidar_distances['front'] < 3 and self.lidar_distances['fleft'] < 3 and self.lidar_distances['fright'] < 3:
                # move back
                pos_output = -1.0
                ang_output = 0.0
                self.publish_data(pos_output, ang_output)
                time.sleep(5)
            elif self.lidar_distances['front'] < 3 and self.lidar_distances['fleft'] < 3 and self.lidar_distances['fright'] > 3:
                # move to the right
                pos_output = 0.0 
                ang_output = -0.3
            elif self.lidar_distances['front'] < 3 and self.lidar_distances['fleft'] > 3 and self.lidar_distances['fright'] < 3:
                # move to the left
                pos_output = 0.0 
                ang_output = 0.3
            elif self.lidar_distances['front'] > 3 and self.lidar_distances['fleft'] < 3 and self.lidar_distances['fright'] > 3:
                # move to the right
                pos_output = 0.0
                ang_output = -0.3
            elif self.lidar_distances['front'] > 3 and self.lidar_distances['fleft'] > 3 and self.lidar_distances['fright'] < 3:
                # move to the left
                pos_output = 0.0 
                ang_output = 0.3
            elif self.lidar_distances['front'] < 9 or self.lidar_distances['fleft'] < 9 or self.lidar_distances['fright'] < 9:
                # move slow
                pos_output, ang_output = self.control(goal_distance, goal_angle)
                pos_output = 0.1 
            elif self.lidar_distances['fleft'] < 3:
                # move to the right
                pos_output = 0.0
                ang_output = -0.5
            elif self.lidar_distances['fright'] < 3:
                # move to the left
                pos_output = 0.0 
                ang_output = 0.5
            else:
                pos_output, ang_output = self.control(goal_distance, goal_angle)
            
            
        else:
            pos_output, ang_output = self.control(goal_distance, goal_angle)
        
        self.publish_data(pos_output, ang_output)

    def control(self, goal_distance, goal_angle):
        self.pos_control.update(goal_distance)
        self.ang_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = self.pos_constrain(- self.pos_control.output/self.dis4constV)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_control.output/180

        return pos_output, ang_output

    def station_keeping(self, goal_distance, goal_angle):
        self.pos_station_control.update(goal_distance)
        self.ang_station_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = self.pos_station_constrain(- self.pos_station_control.output/self.dis4constV)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_station_control.output/180

        # if the goal is behind the robot
        if goal_distance < self.robot_radius:
            pos_output = 0
            ang_output = ang_output
        elif abs(goal_angle) > 90 and goal_distance < 1.5: 
            pos_output = - pos_output
            ang_output = - ang_output

        return pos_output, ang_output

    def publish_data(self, pos_output, ang_output):
        self.cmd_drive.linear.x = pos_output
        self.cmd_drive.linear.y = 0.0
        self.cmd_drive.linear.z = 0.0

        self.cmd_drive.angular.x = 0.0
        self.cmd_drive.angular.y = 0.0
        self.cmd_drive.angular.z = ang_output

        self.pub_cmd_vel.publish(self.cmd_drive)

    def cb_goal(self, msg):
        # print(msg)
        if msg.header.frame_id != self.frame:
            self.goal = None
            print(self.goal)
            return

        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        # print(self.goal)

        if self.goal[0] != -553:
            self.docking = False
            # print(self.docking)
        elif self.goal[0] == -553:
            self.docking = True
            # print(self.docking)

    def initialize_PID(self):
        self.pos_control.setSampleTime(1)
        self.ang_control.setSampleTime(1)
        self.pos_station_control.setSampleTime(1)
        self.ang_station_control.setSampleTime(1)

        self.pos_control.SetPoint = 0.0
        self.ang_control.SetPoint = 0.0
        self.pos_station_control.SetPoint = 0.0
        self.ang_station_control.SetPoint = 0.0

    def cb_collision(self, msg):
        if self.collision_states == True:
            if msg.states == [] and self.count > 1000:
                self.collision_states = False
                # print(self.collision_states)
            else:
                self.count += 1
                # print(self.count)
        # elif msg.states != [] and self.count == 0:
            # pos_output = -1.0
            # ang_output = 0.0
            # self.publish_data(pos_output, ang_output)
            # time.sleep(10)
        else:
            self.collision_states = False
            self.count = 0
        

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_goal_angle(self, robot_yaw, robot, goal):
        robot_angle = np.degrees(robot_yaw)
        p1 = [robot[0], robot[1]]
        p2 = [robot[0], robot[1] + 1.0]
        p3 = goal
        angle = self.get_angle(p1, p2, p3)
        result = angle - robot_angle
        result = self.angle_range(-(result + 90.))
        return result

    def get_angle(self, p1, p2, p3):
        v0 = np.array(p2) - np.array(p1)
        v1 = np.array(p3) - np.array(p1)
        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        return np.degrees(angle)

    def get_scan(self, msg):
        samples = len(msg.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        
        # print samples
        # print 'Value at 0 degrees (Right side or Starboard side)'  
        # print msg.ranges[0]
        # print 'Value at 90 degrees ()'
        # print msg.ranges[360]
        # print 'Value at 180 degrees (Left side or Port side)'
        # print msg.ranges[718]

        self.lidar_distances = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:718]), 10),
        }

        print(self.lidar_distances['front'], self.lidar_distances['fright'], self.lidar_distances['fleft'])

    def angle_range(self, angle): # limit the angle to the range of [-180, 180]
        if angle > 180:
            angle = angle - 360
            angle = self.angle_range(angle)
        elif angle < -180:
            angle = angle + 360
            angle = self.angle_range(angle)
        return angle

    def pos_constrain(self, input):
        if input > self.pos_ctrl_max:
            return self.pos_ctrl_max

        if input < self.pos_ctrl_min:
            return self.pos_ctrl_min

        return input

    def pos_station_constrain(self, input):
        if input > self.pos_station_max:
            return self.pos_station_max
            
        if input < self.pos_station_min:
            return self.pos_station_min

        return input

    def cb_pos_pid(self, config, level):
        print("Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_control.setKp(Kp)
        self.pos_control.setKi(Ki)
        self.pos_control.setKd(Kd)
        return config

    def cb_ang_pid(self, config, level):
        print("Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_control.setKp(Kp)
        self.ang_control.setKi(Ki)
        self.ang_control.setKd(Kd)
        return config

    def cb_pos_station_pid(self, config, level):
        print("Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_station_control.setKp(Kp)
        self.pos_station_control.setKi(Ki)
        self.pos_station_control.setKd(Kd)
        return config

    def cb_ang_station_pid(self, config, level):
        print("Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_station_control.setKp(Kp)
        self.ang_station_control.setKi(Ki)
        self.ang_station_control.setKd(Kd)
        return config

    def cb_joy(self, msg):
        start_button = 7
        back_button = 6
        Y = 3   # start_station_keeping = true
        X = 2   # start_station_keeping = false

        if (msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            self.start_navigation = True
            rospy.loginfo('go auto')
        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            self.start_navigation = False
            rospy.loginfo('go manual')

        if (msg.buttons[Y] == 1) and not self.start_station_keeping:
            self.final_goal = self.goal
            self.start_station_keeping = True
            rospy.loginfo('start station keeping')
        elif msg.buttons[X] == 1 and self.start_station_keeping:
            self.final_goal = None
            self.start_station_keeping = False
            rospy.loginfo('start navigation')

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('waypoint_navigation_obstacle_node',anonymous=False)
    waypoint_navigation_obstacle_node = waypoint_navigation_obstacle()
    rospy.on_shutdown(waypoint_navigation_obstacle_node.on_shutdown)
    rospy.spin()