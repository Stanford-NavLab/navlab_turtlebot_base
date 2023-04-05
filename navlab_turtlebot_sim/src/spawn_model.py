#!/usr/bin/env python

import sys

import tf
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

def spawn_model(robot_name):
    """Spawns robot using starting position from ROS params

    Parameters
    ----------
    robot_name : string
        Name for the gazebo model and parameter name.

    """

    print("input params:",robot_name)

    # remove slashes from robot_name if they exist
    if robot_name[0] == '/': robot_name = robot_name[1:]
    if robot_name[-1] == '/': robot_name = robot_name[:-1]

    init_params = ["start_x","start_y","start_yaw"]

    while True:
        if all([rospy.has_param('/' + robot_name + '/' + x) for x in init_params]):
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
            try:
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                model_xml = rospy.get_param("/robot_description")

                initial_pose = Pose()
                start_x = rospy.get_param('/' + robot_name + '/' + "start_x")
                initial_pose.position.x = start_x
                start_y = rospy.get_param('/' + robot_name + '/' + "start_y")
                initial_pose.position.y = start_y
                start_yaw = rospy.get_param('/' + robot_name + '/' + "start_yaw")
                quaternion = tf.transformations.quaternion_from_euler(0.,0.,start_yaw)
                initial_pose.orientation.x = quaternion[0]
                initial_pose.orientation.y = quaternion[1]
                initial_pose.orientation.z = quaternion[2]
                initial_pose.orientation.w = quaternion[3]

                resp = spawn_model(robot_name,
                                   model_xml,
                                   robot_name+"/",
                                   initial_pose,
                                    "world",
                                   )
                break
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            print("waiting for %s parameters"%(robot_name))
            rospy.sleep(1)

if __name__ == "__main__":
    print("args here:",sys.argv)
    if len(sys.argv) <= 2:
        print("navlab spawn model called without name argument")
    else:
        spawn_model(sys.argv[1])
