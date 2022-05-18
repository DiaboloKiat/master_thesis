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
        self.two_pose = [0.0, 0.0, 0.0]
        self.motion_turn = True

        #get uwb anchors position
        self.sensor_pos = []
        self.sensor_pos = self.get_anchors_pos()

        # Subscriber
        # sub_distance = message_filters.Subscriber("uwb_data_distance", uwb_data)
        # sub_distance_ = message_filters.Subscriber("uwb_data_distance_", uwb_data)
        # ats = ApproximateTimeSynchronizer((sub_distance, sub_distance_), queue_size = 10, slop = 0.1, allow_headerless = True)
        # ats.registerCallback(self.cb_sub_data)
        rospy.Subscriber("uwb_data_distance", uwb_data, self.subscribe_data, queue_size=1)

        self.pub_data_two = rospy.Publisher('localization_data_topic', PoseStamped, queue_size=10)

    def position_calculation(self): 
        uwb_transient_id = []
        uwb_transient_distance = []
        sensor_pose_transient = []

        count = 0
        for i in range(len(self.all_destination_id)):
            uwb_id = self.all_destination_id[i]
            uwb_range = self.all_distance[i]

            if not np.isnan(uwb_range):
                if count < 2 and uwb_range < 21000:
                    if i == 0:
                        if self.all_distance[2] < 20000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 1:
                        if self.all_distance[3] < 7000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 2:
                        if self.all_distance[4] < 20000 or self.all_distance[5] < 18000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 3:
                        if self.all_distance[6] < 20000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 4:
                        if self.motion_turn:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 5:
                        if self.all_distance[7] < 13000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 6:
                        if self.all_distance[9] < 21000:
                            pass
                        else:
                            uwb_transient_id.append(uwb_id) 
                            uwb_transient_distance.append(uwb_range)
                            sensor_pose_transient.append(self.sensor_pos[i])
                    
                            if count == 0:
                                self.x_1 = self.sensor_pos[i][0] * 1000
                                self.y_1 = self.sensor_pos[i][1] * 1000
                            elif count == 1:
                                self.x_2 = self.sensor_pos[i][0] * 1000
                                self.y_2 = self.sensor_pos[i][1] * 1000
                            count += 1
                    elif i == 7:
                        uwb_transient_id.append(uwb_id) 
                        uwb_transient_distance.append(uwb_range)
                        sensor_pose_transient.append(self.sensor_pos[i])
                
                        if count == 0:
                            self.x_1 = self.sensor_pos[i][0] * 1000
                            self.y_1 = self.sensor_pos[i][1] * 1000
                        elif count == 1:
                            self.x_2 = self.sensor_pos[i][0] * 1000
                            self.y_2 = self.sensor_pos[i][1] * 1000
                        count += 1
                    elif i == 8:
                        pass
                    else:
                        uwb_transient_id.append(uwb_id) 
                        uwb_transient_distance.append(uwb_range)
                        sensor_pose_transient.append(self.sensor_pos[i])
                
                        if count == 0:
                            self.x_1 = self.sensor_pos[i][0] * 1000
                            self.y_1 = self.sensor_pos[i][1] * 1000
                            count += 1
                        elif count == 1:
                            self.x_2 = self.sensor_pos[i][0] * 1000
                            self.y_2 = self.sensor_pos[i][1] * 1000
                        count += 1
                        
    
        if count == 2:
            if sensor_pose_transient[0][0] > sensor_pose_transient[1][0]:
                x_max = sensor_pose_transient[0][0] * 1000
                x_min = sensor_pose_transient[1][0] * 1000
            else:
                x_max = sensor_pose_transient[1][0] * 1000
                x_min = sensor_pose_transient[0][0] * 1000

            if sensor_pose_transient[0][1] > sensor_pose_transient[1][1]:
                y_max = sensor_pose_transient[0][1] * 1000
                y_min = sensor_pose_transient[1][1] * 1000
            else:
                y_max = sensor_pose_transient[1][1] * 1000
                y_min = sensor_pose_transient[0][1] * 1000

            if (x_max - x_min) > (y_max - y_min):
                a = uwb_transient_distance[0]
                c = uwb_transient_distance[1]

                a_ = ( (a / (a+c)) * (x_max - x_min) )
                c_ = ( (c / (a+c)) * (x_max - x_min) )
                # print(a_, c_)

                if y_max > 0:
                    self.two_pose[1] = (y_max - (y_max - y_min)/2) + 0.00001
                else:
                    self.two_pose[1] = ((y_max - y_min)/2 + y_min) + 0.00001

                self.two_pose[0] = x_max - a_
                self.two_pose[2] = 0.0
            else:
                a = uwb_transient_distance[0]
                c = uwb_transient_distance[1]

                a_ = ( (a / (a+c)) * (y_max - y_min) )
                c_ = ( (c / (a+c)) * (y_max - y_min) )
                # print(a_, c_)
                if x_max > 0:
                    self.two_pose[0] = (x_max - (x_max - x_min)/2) + 0.00001
                else:
                    self.two_pose[0] = ((x_max - x_min)/2 + x_min) + 0.00001

                self.two_pose[1] = y_max - c_
                self.two_pose[2] = 0.0     

            self.publish_data_two(self.two_pose[0], self.two_pose[1], self.two_pose[2])
        else:
            self.two_pose[0] = np.nan
            self.two_pose[1] = np.nan 
            self.two_pose[2] = np.nan 
            self.publish_data_two(self.two_pose[0], self.two_pose[1], self.two_pose[2])

        

    def publish_data_two(self, pose_x, pose_y, pose_z):
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
        self.pub_data_two.publish(robot_pos)

    def subscribe_data(self, uwb_data_cell):
        self.all_destination_id = uwb_data_cell.destination_id
        self.all_distance = uwb_data_cell.distance
        self.position_calculation()
    
    # def subscribe_data_(self, uwb_data_cell):
    #     self.all_destination_id = uwb_data_cell.destination_id
    #     self.all_distance_ = uwb_data_cell.distance
        

    # def cb_sub_data(self, msg_distance, msg_distance_):
    #     self.subscribe_data(msg_distance)
    #     self.subscribe_data_(msg_distance_)
    #     self.position_calculation()

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