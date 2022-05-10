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

from PID import PID_control

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.05
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class waypoint_navigation_obstacle():
    def __init__(self):
        self.node_name = rospy.get_name()

        # initiallize boat status
        self.auto = 0
        self.start_station_keeping = False
        self.start_navigation = False

        self.robot_pose = [0.0, 0.0]
        self.stop_pose = []
        self.cmd_drive = Twist()
        self.robot_radius = 0.25
        self.yaw = 0
        self.lidar_distances = [0.0, 0.0]

        self.points = queue.Queue(maxsize=20)
        self.reverse = False
        
        self.pt_list = [(2.5, 2.5),(2.5, -2.5),(-2.5, -2.5),(-2.5, 2.5)]
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
        
        self.pub_cmd_vel = rospy.Publisher("joy_teleop/cmd_vel", Twist, queue_size=1)
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.Markers = MarkerArray()
        self.Markers.markers = []
        

        self.sub_joy = rospy.Subscriber("joy_teleop/joy", Joy, self.cb_joy, queue_size=1)
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.get_scan, queue_size=1)
        self.sub = rospy.Subscriber("odometry", Odometry, self.cb_odom, queue_size=1)

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

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        self.pub_points.publish(self.Markers)

        if self.auto == 0:
            return

        quaternion = (msg.pose.pose.orientation.x,\
                        msg.pose.pose.orientation.y,\
                        msg.pose.pose.orientation.z,\
                        msg.pose.pose.orientation.w)

        _, _, self.yaw = tf.transformations.euler_from_quaternion(quaternion)

        goal_angle = self.get_goal_angle(self.yaw, self.robot_pose, self.goal)

        min_distance = min(self.lidar_distances)
        print(min_distance)

        if min_distance < SAFE_STOP_DISTANCE:
            pos_output = 0.0
            ang_output = 0.0
            rospy.loginfo('Stop!')
        elif goal_distance < self.station_keeping_distance or self.start_station_keeping:
            pos_output, ang_output = self.station_keeping(goal_distance, goal_angle)
        else:
            pos_output, ang_output = self.control(goal_distance, goal_angle)
            rospy.loginfo('Distance of the obstacle : %f', min_distance)

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

    def initialize_PID(self):
        self.pos_control.setSampleTime(1)
        self.ang_control.setSampleTime(1)
        self.pos_station_control.setSampleTime(1)
        self.ang_station_control.setSampleTime(1)

        self.pos_control.SetPoint = 0.0
        self.ang_control.SetPoint = 0.0
        self.pos_station_control.SetPoint = 0.0
        self.ang_station_control.SetPoint = 0.0

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
        scan_filter = []
       
        samples = len(msg.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.

        samples_view = 1            # 1 <= samples_view <= samples
        
        print samples
        print 'Value at 0 degrees'
        print msg.ranges[0]
        print 'Value at 90 degrees'
        print msg.ranges[360]
        print 'Value at 180 degrees'
        print msg.ranges[718]

        if samples_view > samples:
            samples_view = samples

        # if samples_view is 1:
        #     scan_filter.append(mgs.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = msg.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = msg.ranges[:right_lidar_samples_ranges]
            print(left_lidar_samples_ranges, right_lidar_samples_ranges)
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        print(scan_filter)
        self.lidar_distances = scan_filter

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
            rospy.loginfo('start_station_keeping')
        elif msg.buttons[X] == 1 and self.start_station_keeping:
            self.final_goal = None
            self.start_station_keeping = False
            rospy.loginfo('start_navigation')

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
    rospy.init_node('waypoint_navigation_obstacle_node',anonymous=False)
    waypoint_navigation_obstacle_node = waypoint_navigation_obstacle()
    rospy.on_shutdown(waypoint_navigation_obstacle_node.on_shutdown)
    rospy.spin()