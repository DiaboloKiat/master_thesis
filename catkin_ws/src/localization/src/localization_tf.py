#! /usr/bin/env python

import rospy
import tf
import math
import numpy as np

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import String
from math import sqrt
from math import pow
from math import atan2
from scipy.stats import norm
from gazebo_msgs.msg import ModelStates

class Locailizationtf():
    def __init__(self):
        self.node_name = rospy.get_name()

        self.fram_name = rospy.get_param("~frame_name", "base_link")
        self.paraent_name = rospy.get_param("~parent_name", "map")

        self.odometry = Odometry()
        self.posestamped = PoseStamped()
        self.broadcaster = tf.TransformBroadcaster()
        self.MODELSTATE_INDEX = int(rospy.get_param("~modelstate_index"))
        rospy.loginfo("%s is %s", rospy.resolve_name('modelstate_index'), self.MODELSTATE_INDEX)
        
        self.pub_posestamped = rospy.Publisher("posestamped_tf", PoseStamped, queue_size=1)
        self.pub_odometry = rospy.Publisher("odometry_tf", Odometry, queue_size=1)
        self.sub_data = rospy.Subscriber("/gazebo/model_states", ModelStates, self.subscribe_data, queue_size=1)

    def subscribe_data(self, ModelStates):
        self.broadcaster.sendTransform((ModelStates.pose[self.MODELSTATE_INDEX].position.x, \
                         ModelStates.pose[self.MODELSTATE_INDEX].position.y, ModelStates.pose[self.MODELSTATE_INDEX].position.z), \
                        (ModelStates.pose[self.MODELSTATE_INDEX].orientation.x, ModelStates.pose[self.MODELSTATE_INDEX].orientation.y, \
                        ModelStates.pose[self.MODELSTATE_INDEX].orientation.z, ModelStates.pose[self.MODELSTATE_INDEX].orientation.w), \
                        rospy.Time.now(), self.fram_name, self.paraent_name)

        self.posestamped.header.stamp = rospy.Time.now()
        self.posestamped.header.frame_id = self.paraent_name
        self.posestamped.pose.position.x = ModelStates.pose[self.MODELSTATE_INDEX].position.x
        self.posestamped.pose.position.y = ModelStates.pose[self.MODELSTATE_INDEX].position.y
        self.posestamped.pose.position.z = ModelStates.pose[self.MODELSTATE_INDEX].position.z
        self.posestamped.pose.orientation.x = ModelStates.pose[self.MODELSTATE_INDEX].orientation.x
        self.posestamped.pose.orientation.y = ModelStates.pose[self.MODELSTATE_INDEX].orientation.y
        self.posestamped.pose.orientation.z = ModelStates.pose[self.MODELSTATE_INDEX].orientation.z
        self.posestamped.pose.orientation.w = ModelStates.pose[self.MODELSTATE_INDEX].orientation.w
        
        # new_pos = np.array(
        #     [ModelStates.pose[self.MODELSTATE_INDEX].position.x, 
        #     ModelStates.pose[self.MODELSTATE_INDEX].position.y, 
        #     ModelStates.pose[self.MODELSTATE_INDEX].position.z])
        # print(new_pos)
        self.pub_posestamped.publish(self.posestamped)

        self.odometry.header.stamp= rospy.Time.now()
        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id = "base_footprint"
        self.odometry.pose.pose.position.x = ModelStates.pose[self.MODELSTATE_INDEX].position.x
        self.odometry.pose.pose.position.y = ModelStates.pose[self.MODELSTATE_INDEX].position.y
        self.odometry.pose.pose.position.z = ModelStates.pose[self.MODELSTATE_INDEX].position.z
        self.odometry.pose.pose.orientation.x = ModelStates.pose[self.MODELSTATE_INDEX].orientation.x
        self.odometry.pose.pose.orientation.y = ModelStates.pose[self.MODELSTATE_INDEX].orientation.y
        self.odometry.pose.pose.orientation.z = ModelStates.pose[self.MODELSTATE_INDEX].orientation.z
        self.odometry.pose.pose.orientation.w = ModelStates.pose[self.MODELSTATE_INDEX].orientation.w


        self.pub_odometry.publish(self.odometry)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('localization_tf_node', anonymous=True)
    localization_tf_node = Locailizationtf()
    rospy.on_shutdown(localization_tf_node.on_shutdown)
    rospy.spin()