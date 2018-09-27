#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def my_spawner(model_path, model_type, model_pose, model_name):
    model_file = open(model_path, 'r')
    model_description = model_file.read()
    if model_type == 'sdf':
        model_spawner_sdf(model_name, model_description, "", model_pose, "world")
        
    elif model_type == 'urdf':
        model_spawner_urdf(model_name, model_description, "", model_pose, "world")

    else:
        print "Model type must be specified"
        sys.exit(0)



rospy.init_node('layout_scratch')

#
# Constants
#
joint_home = [0.0, -math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0]
sqrt_one_half = 0.707106781186547
wall_height = 0.6096
robot_base_height = 0.1
gripper_offset = 0.16385  # Distance from tool flange to gripping screws
hole_offset = 0.00953  # Distance from wall top to hole center
gripper_open_dist = 0.1  #0.0837  # Distance between gripper pads when Robotiq fully open

model_frequency = 1  # Rate at which models are spawned (in Hz)

#
# Wall and footer positions
#
place_footers_x = 0.53033
place_footers_y = 1.79398

place_wall_1_y = 20 * 0.04398

wall_section_length = 20 * 0.03048
footer_height = 0.01905
wall_section_x0 = 20 * 0.04398
wall_section_y0 = 20 * 0.02652
wall_section_y1 = 20 * 0.15288

wall_section_x2 = 20 * 0.17892
wall_section_y2 = 20 * 0.17034

wall_section_x3 = 20 * 0.19638
wall_section_y3 = 20 * 0.21829



# Arrays of names and poses for models
wall_names = ['wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6', 'wall_7', 'wall_8', 'wall_9', 'wall_10', 'wall_11', 'wall_12', 'wall_13', 'wall_14']

wall_poses = [Pose(Point(place_footers_x, place_wall_1_y, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(place_footers_x, place_wall_1_y + wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(place_footers_x, place_wall_1_y + 2 * wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(place_footers_x, place_wall_1_y + 3 * wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(wall_section_x0, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x0 + wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x0 + 2 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x0 + 3 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x0 + 4 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x2, wall_section_y2, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(wall_section_x2, wall_section_y2 + wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)),
               Pose(Point(wall_section_x3, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x3 + wall_section_length, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1)),
               Pose(Point(wall_section_x3 + 2 * wall_section_length, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1))]

# To spawn and delete models
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
model_spawner_sdf = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
model_spawner_urdf = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
model_deleter = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

# Spawn footer boards
# my_spawner(model_path, model_type, model_pose, model_name):
my_spawner('/home/mqm/.gazebo/models/footers/model.sdf', 'sdf', Pose(Point(place_footers_x, place_footers_y, 0), Quaternion(0, 0, sqrt_one_half, sqrt_one_half)), 'footers')

# Spawn wall sections
model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_footers_x, place_wall_1_y, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_1", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_door/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_footers_x, place_wall_1_y + wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_2", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_footers_x, place_wall_1_y + 2 * wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_3", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain_no_holes/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_footers_x, place_wall_1_y + 3 * wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_4", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking_on_left/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x0, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_5", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x0 + wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_6", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x0 + 2 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_7", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x0 + 3 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_8", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain_no_holes/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x0 + 4 * wall_section_length, wall_section_y1, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_9", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain_no_holes/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x2, wall_section_y2, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_10", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x2, wall_section_y2 + wall_section_length, footer_height), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner_sdf("wall_11", model_sdf, "", model_pose, "world")

wall_section_x3 = 20 * 0.19638
wall_section_y3 = 20 * 0.21829

model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking_on_left/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x3, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_12", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_window/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x3 + wall_section_length, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_13", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking_on_right/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_section_x3 + 2 * wall_section_length, wall_section_y3, footer_height), Quaternion(0, 0, 0, 1))
model_spawner_sdf("wall_14", model_sdf, "", model_pose, "world")

roof_x = 0.45413
roof_y = 1.83843
roof_z = 0.62865

model_file = open('/home/mqm/.gazebo/models/roof/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(roof_x, roof_y, roof_z), Quaternion(0, 0, 0, 1))
model_spawner_sdf("roof", model_sdf, "", model_pose, "world")

moveit_commander.roscpp_shutdown()
