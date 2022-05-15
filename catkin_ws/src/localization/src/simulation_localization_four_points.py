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
        self.pose = [0.0, 0.0, 0.0]

        #get uwb anchors position
        self.sensor_pos = []
        self.sensor_pos = self.get_anchors_pos()

        # Subscriber
        rospy.Subscriber("uwb_data_distance", uwb_data, self.subscribe_data, queue_size=1)

        # self.pub_data = rospy.Publisher('localization_data_topic', PoseStamped, queue_size=10)
        self.pub_data = rospy.Publisher('localization_data_topic', PoseStamped, queue_size=10)

    def position_calculation(self):
        self.uwb_transient_id = []
        self.uwb_transient_distance = []
        self.sensor_pose_transient = []

        i = 0

        if not np.isnan(self.all_distance[0]) and not np.isnan(self.all_distance[1]) and \
                not np.isnan(self.all_distance[2]) and not np.isnan(self.all_distance[3]):
            x_1 = self.sensor_pos[3][0] * 1000
            y_1 = self.sensor_pos[3][1] * 1000
            a = self.all_distance[3]

            x_2 = self.sensor_pos[1][0] * 1000
            y_2 = self.sensor_pos[1][1] * 1000
            b = self.all_distance[1]

            x_3 = self.sensor_pos[0][0] * 1000
            y_3 = self.sensor_pos[0][1] * 1000
            c = self.all_distance[0]

            x_4 = self.sensor_pos[2][0] * 1000
            y_4 = self.sensor_pos[2][1] * 1000
            d = self.all_distance[2]

            if b < a and c < a:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0
            else:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_4) ) - ( (y_1 - y_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_4) - (x_2 - x_4)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_4) ) - ( (x_1 - x_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_4) - (y_2 - y_4)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0

        elif not np.isnan(self.all_distance[4]) and not np.isnan(self.all_distance[5]) and  \
                not np.isnan(self.all_distance[6]) and not np.isnan(self.all_distance[7]):
            x_1 = self.sensor_pos[4][0] * 1000
            y_1 = self.sensor_pos[4][1] * 1000
            a = self.all_distance[4]

            x_2 = self.sensor_pos[5][0] * 1000
            y_2 = self.sensor_pos[5][1] * 1000
            b = self.all_distance[5]

            x_3 = self.sensor_pos[6][0] * 1000
            y_3 = self.sensor_pos[6][1] * 1000
            c = self.all_distance[6]

            x_4 = self.sensor_pos[7][0] * 1000
            y_4 = self.sensor_pos[7][1] * 1000
            d = self.all_distance[7]

            if b < a and c < a:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0
            else:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_4) ) - ( (y_1 - y_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_4) - (x_2 - x_4)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_4) ) - ( (x_1 - x_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_4) - (y_2 - y_4)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0

        elif len(self.all_destination_id) < 18:
            print(len(self.all_destination_id))
            self.pose[0] = np.nan
            self.pose[1] = np.nan
            self.pose[2] = np.nan

        elif not np.isnan(self.all_distance[2]) and not np.isnan(self.all_distance[3]) and  \
                not np.isnan(self.all_distance[4]) and not np.isnan(self.all_distance[5]):
            x_1 = self.sensor_pos[5][0] * 1000
            y_1 = self.sensor_pos[5][1] * 1000
            a = self.all_distance[5]

            x_2 = self.sensor_pos[3][0] * 1000
            y_2 = self.sensor_pos[3][1] * 1000
            b = self.all_distance[3]

            x_3 = self.sensor_pos[2][0] * 1000
            y_3 = self.sensor_pos[2][1] * 1000
            c = self.all_distance[2]

            x_4 = self.sensor_pos[4][0] * 1000
            y_4 = self.sensor_pos[4][1] * 1000
            d = self.all_distance[4]

            if b < a and c < a:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0
            else:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_4) ) - ( (y_1 - y_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_4) - (x_2 - x_4)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_4) ) - ( (x_1 - x_2)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_4) - (y_2 - y_4)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0
        
        elif not np.isnan(self.all_distance[4]) and not np.isnan(self.all_distance[5]) and  \
                not np.isnan(self.all_distance[6]):
            x_1 = self.sensor_pos[6][0] * 1000
            y_1 = self.sensor_pos[6][1] * 1000
            a = self.all_distance[6]

            x_2 = self.sensor_pos[5][0] * 1000
            y_2 = self.sensor_pos[5][1] * 1000
            b = self.all_distance[5]

            x_3 = self.sensor_pos[4][0] * 1000
            y_3 = self.sensor_pos[4][1] * 1000
            c = self.all_distance[4]

            self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
            self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
            self.pose[2] = 0.0

        elif not np.isnan(self.all_distance[9]) and not np.isnan(self.all_distance[10]) and  \
                not np.isnan(self.all_distance[11]) and not np.isnan(self.all_distance[12]):
            x_1 = self.sensor_pos[10][0] * 1000
            y_1 = self.sensor_pos[10][1] * 1000
            a = self.all_distance[10]

            x_2 = self.sensor_pos[9][0] * 1000
            y_2 = self.sensor_pos[9][1] * 1000
            b = self.all_distance[9]

            x_3 = self.sensor_pos[11][0] * 1000
            y_3 = self.sensor_pos[11][1] * 1000
            c = self.all_distance[11]

            x_4 = self.sensor_pos[12][0] * 1000
            y_4 = self.sensor_pos[12][1] * 1000
            d = self.all_distance[12]

            if a < c and b < c:
                self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_2 - y_3) ) - ( (y_1 - y_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (x_1 - x_2)*(y_2 - y_3) - (x_2 - x_3)*(y_1- y_2) ) ) )
                self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_2 - x_3) ) - ( (x_1 - x_2)*(c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2) ) ) / ( 2*( (y_1 - y_2)*(x_2 - x_3) - (y_2 - y_3)*(x_1- x_2) ) ) )
                self.pose[2] = 0.0
            else:
                self.pose[0] = ( ( ( (c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2)*(y_2 - y_4) ) - ( (y_2 - y_3)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (x_2 - x_3)*(y_2 - y_4) - (x_2 - x_4)*(y_2- y_3) ) ) )
                self.pose[1] = ( ( ( (c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2)*(x_2 - x_4) ) - ( (x_2 - x_3)*(d**2 - b**2 + x_2**2 - x_4**2 + y_2**2 - y_4**2) ) ) / ( 2*( (y_2 - y_3)*(x_2 - x_4) - (y_2 - y_4)*(x_2- x_3) ) ) )
                self.pose[2] = 0.0

        elif not np.isnan(self.all_distance[6]) and not np.isnan(self.all_distance[9]) and  \
                not np.isnan(self.all_distance[10]):
            x_2 = self.sensor_pos[6][0] * 1000
            y_2 = self.sensor_pos[6][1] * 1000
            b = self.all_distance[6]

            x_3 = self.sensor_pos[9][0] * 1000
            y_3 = self.sensor_pos[9][1] * 1000
            c = self.all_distance[9]

            x_4 = self.sensor_pos[10][0] * 1000
            y_4 = self.sensor_pos[10][1] * 1000
            d = self.all_distance[10]

            self.pose[0] = ( ( ( (c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2)*(y_3 - y_4) ) - ( (y_2 - y_3)*(d**2 - c**2 + x_3**2 - x_4**2 + y_3**2 - y_4**2) ) ) / ( 2*( (x_2 - x_3)*(y_3 - y_4) - (x_3 - x_4)*(y_2- y_3) ) ) )
            self.pose[1] = ( ( ( (c**2 - b**2 + x_2**2 - x_3**2 + y_2**2 - y_3**2)*(x_3 - x_4) ) - ( (x_2 - x_3)*(d**2 - c**2 + x_3**2 - x_4**2 + y_3**2 - y_4**2) ) ) / ( 2*( (y_2 - y_3)*(x_3 - x_4) - (y_3 - y_4)*(x_2- x_3) ) ) )
            self.pose[2] = 0.0
        
        elif not np.isnan(self.all_distance[11]) and not np.isnan(self.all_distance[12]) and  \
                not np.isnan(self.all_distance[13]):
            x_1 = self.sensor_pos[12][0] * 1000
            y_1 = self.sensor_pos[12][1] * 1000
            a = self.all_distance[12]

            x_2 = self.sensor_pos[11][0] * 1000
            y_2 = self.sensor_pos[11][1] * 1000
            b = self.all_distance[11]

            x_4 = self.sensor_pos[13][0] * 1000
            y_4 = self.sensor_pos[13][1] * 1000
            d = self.all_distance[13]

            self.pose[0] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(y_1 - y_4) ) - ( (y_1 - y_2)*(d**2 - a**2 + x_1**2 - x_4**2 + y_1**2 - y_4**2) ) ) / ( 2*( (x_1 - x_2)*(y_1 - y_4) - (x_1 - x_4)*(y_1- y_2) ) ) )
            self.pose[1] = ( ( ( (b**2 - a**2 + x_1**2 - x_2**2 + y_1**2 - y_2**2)*(x_1 - x_4) ) - ( (x_1 - x_2)*(d**2 - a**2 + x_1**2 - x_4**2 + y_1**2 - y_4**2) ) ) / ( 2*( (y_1 - y_2)*(x_1 - x_4) - (y_1 - y_4)*(x_1- x_2) ) ) )
            self.pose[2] = 0.0

        elif not np.isnan(self.all_distance[13]) and not np.isnan(self.all_distance[16]) and  \
                not np.isnan(self.all_distance[17]):
            x_1 = self.sensor_pos[16][0] * 1000
            y_1 = self.sensor_pos[16][1] * 1000
            a = self.all_distance[16]

            x_3 = self.sensor_pos[13][0] * 1000
            y_3 = self.sensor_pos[13][1] * 1000
            c = self.all_distance[13]

            x_4 = self.sensor_pos[17][0] * 1000
            y_4 = self.sensor_pos[17][1] * 1000
            d = self.all_distance[17]

            self.pose[0] = ( ( ( (c**2 - a**2 + x_1**2 - x_3**2 + y_1**2 - y_3**2)*(y_3 - y_4) ) - ( (y_1 - y_3)*(d**2 - c**2 + x_3**2 - x_4**2 + y_3**2 - y_4**2) ) ) / ( 2*( (x_1 - x_3)*(y_3 - y_4) - (x_3 - x_4)*(y_1- y_3) ) ) )
            self.pose[1] = ( ( ( (c**2 - a**2 + x_1**2 - x_3**2 + y_1**2 - y_3**2)*(x_3 - x_4) ) - ( (x_1 - x_3)*(d**2 - c**2 + x_3**2 - x_4**2 + y_3**2 - y_4**2) ) ) / ( 2*( (y_1 - y_3)*(x_3 - x_4) - (y_3 - y_4)*(x_1- x_3) ) ) )
            self.pose[2] = 0.0

        else:
            self.pose[0] = np.nan
            self.pose[1] = np.nan
            self.pose[2] = np.nan

        self.publish_data(self.pose[0], self.pose[1], self.pose[2])

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