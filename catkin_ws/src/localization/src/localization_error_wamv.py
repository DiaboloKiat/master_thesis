#!/usr/bin/env python

import rospy
import tf 
import math
import numpy as np
import time
import threading
import os
import sys
import random
import message_filters

from math import sqrt
from math import pow
from std_msgs.msg import Float64
from pozyx_simulation.msg import  uwb_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer

class localization_error(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.robot_pose_x = 0
        self.robot_pose_y = 0
        self.robot_pose_z = 0

        self.robot_pose_tf_x = 0
        self.robot_pose_tf_y = 0
        self.robot_pose_tf_z = 0

        self.pub_error = rospy.Publisher("localization_error", Float64, queue_size=1)

        # Subscriber
        sub_robot_pose = rospy.Subscriber("/wamv_1/posestamped", PoseStamped, self.cb_robot_pose, queue_size=1)
        sub_robot_pose_tf = rospy.Subscriber("/wamv_1/posestamped", PoseStamped, self.cb_robot_pose_tf, queue_size=1)

    def cb_robot_pose(self, msg_robot_pose):
        self.robot_pose_x = msg_robot_pose.pose.position.x
        self.robot_pose_y = msg_robot_pose.pose.position.y
        self.robot_pose_z = msg_robot_pose.pose.position.z
        print(self.robot_pose_x, self.robot_pose_y, self.robot_pose_z)

        self.error_calculation()

    def cb_robot_pose_tf(self, msg_robot_pose_tf):
        self.robot_pose_tf_x = msg_robot_pose_tf.pose.position.x 
        self.robot_pose_tf_y = msg_robot_pose_tf.pose.position.y
        self.robot_pose_tf_z = msg_robot_pose_tf.pose.position.z
        print(self.robot_pose_tf_x, self.robot_pose_tf_y, self.robot_pose_tf_z)

        # self.error_calculation()

    def error_calculation(self):
        error = math.sqrt(math.pow(self.robot_pose_tf_x - self.robot_pose_x, 2) + math.pow(self.robot_pose_tf_y - self.robot_pose_tf_y, 2)) * 1000
        print(error)
        self.pub_error.publish(error)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == "__main__":
    rospy.init_node('localization_error', anonymous=True)

    error = localization_error()
    rospy.spin()
