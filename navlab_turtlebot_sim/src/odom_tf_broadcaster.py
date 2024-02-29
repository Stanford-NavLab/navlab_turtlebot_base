#!/usr/bin/env python

import sys

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def odom_callback(msg, odom_args):
    robot_name, odom_source = odom_args
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = robot_name + "_tf/odom_" + odom_source
    t.child_frame_id = robot_name + "_tf/base_footprint"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    # print("pub:",robot_name,t.header.stamp)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    if len(sys.argv) < 3:
        print("odom_tf_broadcaster called without name and odom_source args")
    else:
        robot_name = sys.argv[1]
        odom_source = sys.argv[2]
        odom_args = (robot_name, odom_source)
        rospy.Subscriber("odom_" + odom_source,
                         Odometry,
                         odom_callback,
                         odom_args
                         )
    rospy.spin()
