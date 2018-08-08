#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_home = [0.0, -math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0]

#
# Initialization portion
#

# First, initialize moveit_commander and rospy.
#print "============ Starting setup"
#moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('hydra_demo')


# Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object.
# This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object.
# This object is an interface to one group of joints.
group_placer = moveit_commander.MoveGroupCommander("placer_manipulator")
group_attacher = moveit_commander.MoveGroupCommander("attacher_manipulator")


# To spawn and delete models
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
model_spawner = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
model_deleter = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
'''
# Spawn a wall
model_file = open('/home/mqm/.gazebo/models/wall_plain/model.sdf', 'r')
wall_sdf = model_file.read()
wall_pose = Pose(Point(0, -0.5, 0.0), Quaternion(0, 0, 0.7071, 0.7071))
model_spawner("wall_1", wall_sdf, "", wall_pose, "world")
'''

# Move the placer manipulator
group_placer.set_joint_value_target(joint_home)
plan = group_placer.plan()
group_placer.execute(plan)
