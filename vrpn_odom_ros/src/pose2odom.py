#!/usr/bin/env python3
"""Republish VRPN PoseStamped messages as Odometry messages

Looks for odometry on topics called: "/vrpn_client_node/turtlebot*/pose"

Publishes goal on topics called: "/turtlebot*/odom"

"""

__authors__ = "D. Knowles"
__date__ = "03 Apr 2023"

import rospy
import numpy as np
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Pose2Odom():
    """Odom republisher.

    """
    def __init__(self):

        # keep track of all pose subscribers
        self.pose_subscribers = set()

        # keep track of all goal publishers
        self.odom_publishers = {}

        # Initialize ROS node.
        rospy.init_node('goal_planner', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(1) # 1Hz

        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()

            # update odom topics
            pose_topics = {x[0] for x in all_topics if "/pose" in x[0]}
            pose_topics = {x for x in pose_topics if "turtlebot" in x}
            pose_topics = {x for x in pose_topics if "vrpn" in x}
            new_pose_topics = pose_topics - self.pose_subscribers
            if len(new_pose_topics) > 0:
                self.subscribe_to_all(new_pose_topics)

            rate.sleep()

    def subscribe_to_all(self, new_topics):
        """Subscribes to all new topics and adds to dictionary.

        Parameters
        ----------
        new_topics : list
            List of topic names for which to subscribe

        """

        for topic_name in new_topics:
            # Subscribe to topic and set callback function
            turtlebot_id = self.get_turtlebot_id(topic_name)

            publish_name = "/turtlebot"+ turtlebot_id + "/odom"
            self.odom_publishers[turtlebot_id] = rospy.Publisher(publish_name,
                                                        Odometry,
                                                        queue_size = 10)

            rospy.Subscriber(topic_name, PoseStamped,
                             callback = self.pose_callback,
                             callback_args = turtlebot_id)
            self.pose_subscribers.add(topic_name)



    def pose_callback(self, data_measured, turtlebot_id):
        """Callback function when new pose messages are published.

        Parameters
        ----------
        data_measured : geometry_msgs/PoseStamped
            Data from the /odom topic that was published.
        turtlebot_id : string
            Turtlebot number for which this callback function was called.

        """

        msg = Odometry()
        msg.header = data_measured.header
        msg.child_frame_id = "turtlebot" + turtlebot_id + "_tf/odom"
        msg.pose.pose = data_measured.pose

        self.odom_publishers[turtlebot_id].publish(msg)


    def get_turtlebot_id(self, topic_name):
        """Get new topic name sorted alphnumerically

        Parameters
        ----------
        topic_name : string
            Current full topic name

        Returns
        -------
        turtlebot_id : string
            Turtlebot identification number
        """

        # remove initial slash if it exists
        if topic_name[0] == "/": topic_name = topic_name[1:]

        turtlebot_name = topic_name.split("/")[1]

        # use whatever follows "turtlebot" as the ID
        turtlebot_id = turtlebot_name[9:]

        return turtlebot_id

    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """

        print("closing safely")


if __name__ == '__main__':
    try:
        p2o = Pose2Odom()
        rospy.on_shutdown(p2o.cleanup)
    except rospy.ROSInterruptException:
        pass
