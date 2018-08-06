#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#
# Initialization portion
#

# First, initialize moveit_commander and rospy.
#print "============ Starting setup"
#moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('hydra_demo')

# To spawn and delete models
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

# Spawn a wall
f = open('/home/mqm/.gazebo/models/wall_plain/model.sdf', 'r')
sdf_wall = f.read()
wall_pose = Pose(Point(0, -0.5, 0.0), Quaternion(0, 0, 0.7071, 0.7071))
s("wall_1", sdf_wall, "", wall_pose, "world")
