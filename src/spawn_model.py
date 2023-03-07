#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

def spawn_model():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        model_name = "turtlebot6"
        model_xml = rospy.get_param("/robot_description")

        initial_pose = Pose()
        initial_pose.position.x = 1.
        initial_pose.position.y = 1.

        resp = spawn_model(model_name,
                           model_xml,
                            "/",
                           initial_pose,
                            "world",
                           )
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    spawn_model()
