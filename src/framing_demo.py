#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_home = [0, -math.pi/2-0.2, math.pi/2, math.pi/2, math.pi/2, 0]
joint_zeros = [0, 0, 0, 0, 0, 0]
sqrt_one_half = 0.707106781186547
wall_height = 0.6096
robot_base_height = 0.1
gripper_offset = 0.16385  # Distance from tool flange to gripping screws
hole_offset = 0.00953  # Distance from wall top to hole center

#gripper_open_dist = 0.1  #0.0837  # Distance between gripper pads when Robotiq fully open
gripper_zero_dist = 0.033  # Distance between gripper pad and center when closed
two_by_width = 0.0889  # 2.5 inches in m
gripper_close_dist = 0.01145  # Distance between gripper and centerline when closing on 2x4
gripper_open_padding = 0.015  # How far extra to move gripper half when open

place_wall_1_x = 0.75145
place_wall_1_y = 0.17961
place_wall_2_x = 0.79696
place_wall_2_y = -0.2688

place_wall_1_x = 0.75145
place_wall_1_y = 0.17961
place_wall_2_x = 0.79696
place_wall_2_y = -0.2688

behavior_selection = sys.argv[1]

rospy.init_node('framing_demo')

if behavior_selection == 'init_joints':
    # Publishers for manipulator joints to circumvent MoveIt!
    pub_joint_attacher = rospy.Publisher('hydra_attacher_arm_controller/command', JointTrajectory, queue_size=10)
    pub_joint_placer = rospy.Publisher('hydra_placer_arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(3)
    jt = JointTrajectory()
    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ""
    jt.joint_names = ['attacher_shoulder_pan_joint', 'attacher_shoulder_lift_joint', 'attacher_elbow_joint', 'attacher_wrist_1_joint', 'attacher_wrist_2_joint', 'attacher_wrist_3_joint']
    p = JointTrajectoryPoint()
    p.positions = [0, -math.pi/2-0.2, math.pi/2, math.pi/2, math.pi/2, 0]
    jt.points.append(p)
    jt.points[0].time_from_start = rospy.Duration.from_sec(2)
    pub_joint_attacher.publish(jt)
    jt.joint_names = ['placer_shoulder_pan_joint', 'placer_shoulder_lift_joint', 'placer_elbow_joint', 'placer_wrist_1_joint', 'placer_wrist_2_joint', 'placer_wrist_3_joint']
    pub_joint_placer.publish(jt)
    #rospy.spin()
    rospy.sleep(3)
    sys.exit(0)

#
# Initialization portion
#

# Publishers for gripper prismatic joints
pub_gripper_p = rospy.Publisher('gripper_placer_controller_p/command', Float64, queue_size=3)
pub_gripper_d = rospy.Publisher('gripper_placer_controller_d/command', Float64, queue_size=3)

if behavior_selection == 'gripper_test':
    pub_gripper_p.publish(0)
    pub_gripper_d.publish(0)
    rospy.sleep(1)
    pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
    pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
    rospy.sleep(1)
    pub_gripper_p.publish(gripper_close_dist)
    pub_gripper_d.publish(-gripper_close_dist)
    sys.exit(0)

# First, initialize moveit_commander and rospy.
moveit_commander.roscpp_initialize(sys.argv)

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
model_spawner_urdf = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
model_deleter = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

#
# Spawn footers and walls
#

# Spawn footer boards
'''
model_file = open('/home/mqm/.gazebo/models/footer_long/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_wall_1_x, place_wall_1_y, 0), Quaternion(0, 0, 0.3827, 0.9239))
model_spawner("footer_1", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/footer_short/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_wall_2_x, place_wall_2_y, 0), Quaternion(0, 0, -0.3827, 0.9239))
model_spawner("footer_2", model_sdf, "", model_pose, "world")
'''
model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/footer_simple_test/footer_long_simple.urdf.xacro', 'r')
model_description = model_file.read()
model_pose = Pose(Point(place_wall_1_x-.02, place_wall_1_y-.02, 0), Quaternion(0, 0, 0.3827, 0.9239))
model_spawner_urdf("footer_1_simple", model_description, "", model_pose, "world")
rospy.sleep(1)
model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/footer_simple_test/footer_simple_test.urdf.xacro', 'r')
model_description = model_file.read()
model_pose = Pose(Point(place_wall_2_x-.03, place_wall_2_y-.03, 0), Quaternion(0, 0, -0.3827, 0.9239))
model_spawner_urdf("footer_2_simple", model_description, "", model_pose, "world")
rospy.sleep(1)

# Spawn walls
model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking/model.sdf', 'r')
model_sdf = model_file.read()
wall_1_x = 0.4
wall_1_y = -0.5
wall_1_z = 0.0
model_pose = Pose(Point(wall_1_x, wall_1_y, wall_1_z), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner("wall_1", model_sdf, "", model_pose, "world")
rospy.sleep(1)

model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/wall_simple_test/wall_simple_test.urdf.xacro', 'r')
model_description = model_file.read()
model_pose = Pose(Point(wall_1_x, wall_1_y, wall_1_z), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
#model_spawner_urdf("wall_1_simple", model_description, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_1_x-0.25, wall_1_y, wall_1_z), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
#model_spawner("wall_2", model_sdf, "", model_pose, "world")

# Move the placer manipulator to home position
group_placer.set_joint_value_target(joint_home)
plan = group_placer.plan()
group_placer.execute(plan)

# Get current pose of placer for reference
pose_c = group_placer.get_current_pose().pose
print "========== Current placer pose: ", pose_c

# Open the gripper
pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
rospy.sleep(1)
pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
rospy.sleep(1)

# Approach pose for grasping first wall
# Orientation is 0, pi, pi/2 = XYZ
grasp_z = wall_1_z + wall_height - robot_base_height + gripper_offset - hole_offset + .007
grasp_approach_z = grasp_z + 0.05
grasp_approach_pose = Pose(Point(wall_1_x, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
waypoints = []
waypoints.append(copy.deepcopy(grasp_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)

# Get current pose of placer for reference
pose_c = group_placer.get_current_pose().pose
print "========== Current placer pose: ", pose_c
    
# Grasp pose for first wall
grasp_pose = copy.deepcopy(grasp_approach_pose)
grasp_pose.position.z = grasp_z
#grasp_pose = Pose(Point(wall_1_x, wall_1_y, grasp_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
waypoints = []
waypoints.append(copy.deepcopy(grasp_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)
rospy.sleep(1)

# Close the gripper
pub_gripper_p.publish(gripper_close_dist)
pub_gripper_d.publish(-gripper_close_dist)
rospy.sleep(3)

# Lift the wall
waypoints = []
waypoints.append(copy.deepcopy(grasp_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)

# Approach pose for placing first wall
# Orientation is 0, pi, 5*pi/4 = XYZ
place_approach_pose = Pose(Point(place_wall_1_x, place_wall_1_y, grasp_approach_z), Quaternion(0.9239, 0.3827, 0, 0))
waypoints = []
waypoints.append(copy.deepcopy(place_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)

# Place the wall
place_pose = Pose(Point(place_wall_1_x, place_wall_1_y, grasp_z+.0127), Quaternion(0.9239, 0.3827, 0, 0))
waypoints = []
waypoints.append(copy.deepcopy(place_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)
rospy.sleep(6)

# Spawn a heavy block for holding the wall in place
model_file = open('/home/mqm/.gazebo/models/heavy_block/model.sdf', 'r')
model_sdf = model_file.read()
#model_pose = Pose(Point(wall_1_x, wall_1_y, .0292), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_pose = Pose(Point(place_wall_1_x, place_wall_1_y, .0292+.01905/2), Quaternion(0, 0, 0.3827, 0.9239))
model_spawner("heavy_block_1", model_sdf, "", model_pose, "world")

# Open the gripper
pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
rospy.sleep(1)
pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
rospy.sleep(3)

# Approach pose for placing first wall
waypoints = []
waypoints.append(copy.deepcopy(place_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)



moveit_commander.roscpp_shutdown()
