#! /usr/bin/env python
import rospy
import tf
from tf import TransformListener, TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class Drawer(object):
    def __init__(self):
        # self.pub_markers = rospy.Publisher('route', MarkerArray, queue_size=1)
        self.pub_path = rospy.Publisher('route', Path, queue_size=1)

        self.marray = MarkerArray()
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.count = 0
        self.transformer = TransformerROS()
        self.sub_topic = rospy.get_param("~sub_topic", "posestamped")
        self.pub_topic = rospy.get_param("~pub_topic", "route")

        sub_pose = rospy.Subscriber(self.sub_topic, PoseStamped, self.to_pub, queue_size=1)

    def to_pub(self, msg):
        # adjustment
        trans = []
        trans.append(0)
        trans.append(0)
        trans.append(0)
        r = 0
        p = 0
        y = 0
        rot = tf.transformations.quaternion_from_euler(r, p, y)

        transpose_matrix = self.transformer.fromTranslationRotation(trans, rot)

        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z, 1])
        new_p = np.dot(transpose_matrix, p)

        rospy.loginfo('pose marker%d' % self.count)
        self.path.header.stamp = rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.header.seq = self.count
        self.count += 1
        pose.pose.position.x = new_p[0]
        pose.pose.position.y = new_p[1]
        pose.pose.position.z = 0
        pose.pose.orientation = msg.pose.orientation

        self.path.poses.append(pose)

        self.pub_path.publish(self.path)


if __name__ == "__main__":
    rospy.init_node("drawer_pose")
    drawer = Drawer()
    rospy.spin()