#!/usr/bin/env python

import rospy 
import numpy as np
import scipy.stats
import math
import tf 
import time
import rostopic
import re
import matplotlib.pyplot as plt 
import message_filters

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from pozyx_simulation.msg import  uwb_data
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer


class simulation_localization():
    def __init__(self):
        self.node_name = rospy.get_name()

        self.all_distance = []
        self.all_destination_id = []
        # self.pose = [0.0, 0.0, 0.0]
        self.three_pose = [0.0, 0.0, 0.0]

        #get uwb anchors position
        self.sensor_pos = []
        self.sensor_pos = self.get_anchors_pos()

        # Subscriber
        rospy.Subscriber("uwb_data_distance", uwb_data, self.subscribe_data, queue_size=1)

        # self.pub_data = rospy.Publisher('localization_data_topic', PoseStamped, queue_size=10)
        self.pub_data_three = rospy.Publisher('localization_data_topic', PoseStamped, queue_size=10)

    def position_calculation(self): 
        for i in range(len(self.all_destination_id)):
            uwb_id = self.all_destination_id[i]
            uwb_range = self.all_distance[i]

            if i == 0:
                x_max = self.sensor_pos[i][0] * 1000
                y_max = self.sensor_pos[i][1] * 1000
                x_1 = self.sensor_pos[i][0] * 1000
                y_1 = self.sensor_pos[i][1] * 1000
                a = uwb_range
            elif i == 1:
                x_2 = self.sensor_pos[i][0] * 1000
                y_2 = self.sensor_pos[i][1] * 1000
                b = uwb_range
            elif i == 2:
                x_min = self.sensor_pos[i][0] * 1000
                y_min = self.sensor_pos[i][1] * 1000
                x_3 = self.sensor_pos[i][0] * 1000
                y_3 = self.sensor_pos[i][1] * 1000
                c = uwb_range
            elif i == 3:
                x_4 = self.sensor_pos[i][0] * 1000
                y_4 = self.sensor_pos[i][1] * 1000
                d = uwb_range

        a_ = (a / (a + c)) * (abs(x_1) + abs(x_3))
        c_ = (c / (a + c)) * (abs(x_1) + abs(x_3))
        print(a, a_, c, c_, x_1 - x_3)

        # self.pose[0] = (b**2 - a**2 + x_max**2 - x_min**2) / (2*(x_max - x_min))
        # self.pose[1] = (c**2 - b**2 + y_max**2 - y_min**2) / (2*(y_max - y_min))
        # self.pose[2] = 0.0

        self.three_pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
        self.three_pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
        self.three_pose[2] = 0.0

        # self.publish_data(self.pose[0], self.pose[1], self.pose[2])
        self.publish_data_three(self.three_pose[0], self.three_pose[1], self.three_pose[2])  

    def publish_data(self, pose_x, pose_y, pose_z):
        robot_pos = PoseStamped()
        robot_pos.pose.position.x = float(pose_x)
        robot_pos.pose.position.y = float(pose_y)
        robot_pos.pose.position.z = float(pose_z)

        robot_pos.pose.orientation.x = 0.0
        robot_pos.pose.orientation.y = 0.0
        robot_pos.pose.orientation.z = 0.0
        robot_pos.pose.orientation.w = 0.0
        robot_pos.header.stamp = rospy.Time.now() 
        robot_pos.header.frame_id = "map" 
        # rospy.loginfo(robot_pos)
        self.pub_data.publish(robot_pos)

    def publish_data_three(self, pose_x, pose_y, pose_z):
        robot_pos = PoseStamped()
        robot_pos.pose.position.x = float(pose_x)
        robot_pos.pose.position.y = float(pose_y)
        robot_pos.pose.position.z = float(pose_z)

        robot_pos.pose.orientation.x = 0.0
        robot_pos.pose.orientation.y = 0.0
        robot_pos.pose.orientation.z = 0.0
        robot_pos.pose.orientation.w = 0.0
        robot_pos.header.stamp = rospy.Time.now() 
        robot_pos.header.frame_id = "map" 
        # rospy.loginfo(robot_pos)
        self.pub_data_three.publish(robot_pos)

    def subscribe_data(self, uwb_data_cell):
        self.all_destination_id = uwb_data_cell.destination_id
        self.all_distance = uwb_data_cell.distance
        self.position_calculation()

    def get_anchors_pos(self):
        max_anchor = 100 
        uwb_id = 'uwb_anchor_'
        listener = tf.TransformListener()

        for i in range(max_anchor):
            try:
                time.sleep(0.3)
                (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
                self.sensor_pos.append(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                break

        if self.sensor_pos == []:
            rospy.logwarn("There is not found any anchors. Function is working again.")    
            self.get_anchors_pos()
    
        return self.sensor_pos

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('simulation_localization_node', anonymous=True)
    
    simulation_localization_node = simulation_localization()
    rospy.on_shutdown(simulation_localization_node.on_shutdown)
    rospy.spin()