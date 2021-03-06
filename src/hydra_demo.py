#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_home = [-math.pi/4, -math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0]
joint_zeros = [0, 0, 0, 0, 0, 0]
sqrt_one_half = 0.707106781186547
wall_height = 0.6096
robot_base_height = 0.1
gripper_offset = 0.16385  # Distance from tool flange to gripping screws
hole_offset = 0.00953  # Distance from wall top to hole center
gripper_open_dist = 0.1  #0.0837  # Distance between gripper pads when Robotiq fully open

place_wall_1_x = 0.75145
place_wall_1_y = 0.17961
place_wall_2_x = 0.79696
place_wall_2_y = -0.2688

behavior_selection = sys.argv[1]

#
# Initialization portion
#

# First, initialize moveit_commander and rospy.
print "============ Starting setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('hydra_demo')

# Publishers for gripper prismatic joints
pub_gripper_p = rospy.Publisher('gripper_placer_controller_p/command', Float64, queue_size=3)
pub_gripper_d = rospy.Publisher('gripper_placer_controller_d/command', Float64, queue_size=3)

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
    p.positions = [0, -0.7, 0, 0, 0, 0]
    jt.points.append(p)
    jt.points[0].time_from_start = rospy.Duration.from_sec(2)
    pub_joint_attacher.publish(jt)
    jt.joint_names = ['placer_shoulder_pan_joint', 'placer_shoulder_lift_joint', 'placer_elbow_joint', 'placer_wrist_1_joint', 'placer_wrist_2_joint', 'placer_wrist_3_joint']
    pub_joint_placer.publish(jt)
    #rospy.spin()
    rospy.sleep(3)
    sys.exit(0)

# Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object.
# This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object.
# This object is an interface to one group of joints.
group_placer = moveit_commander.MoveGroupCommander("placer_manipulator")
group_attacher = moveit_commander.MoveGroupCommander("attacher_manipulator")


if behavior_selection == 'move_home':
    # Move the placer manipulator to home position
    group_placer.set_joint_value_target(joint_home)
    plan = group_placer.plan()
    group_placer.execute(plan)
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

# To spawn and delete models
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
model_spawner = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
model_deleter = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

# Spawn footer boards
model_file = open('/home/mqm/.gazebo/models/footer_long/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_wall_1_x, place_wall_1_y, 0), Quaternion(0, 0, 0.3827, 0.9239))
model_spawner("footer_1", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/footer_short/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(place_wall_2_x, place_wall_2_y, 0), Quaternion(0, 0, -0.3827, 0.9239))
model_spawner("footer_2", model_sdf, "", model_pose, "world")

# Spawn walls
model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking/model.sdf', 'r')
model_sdf = model_file.read()
wall_1_x = 0.0
wall_1_y = -0.5
wall_1_z = 0.0
model_pose = Pose(Point(wall_1_x, wall_1_y, wall_1_z), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
model_spawner("wall_1", model_sdf, "", model_pose, "world")

model_file = open('/home/mqm/.gazebo/models/wall_plain/model.sdf', 'r')
model_sdf = model_file.read()
model_pose = Pose(Point(wall_1_x-0.25, wall_1_y, wall_1_z), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
#model_spawner("wall_2", model_sdf, "", model_pose, "world")

# Move the placer manipulator to home position
group_placer.set_joint_value_target(joint_home)
plan = group_placer.plan()
group_placer.execute(plan)

# Get current pose of placer for reference
rospy.sleep(3)
pose_c = group_placer.get_current_pose().pose
print "========== Current placer pose: ", pose_c

# Open the gripper
pub_gripper_p.publish(gripper_open_dist / 2)
pub_gripper_d.publish(-gripper_open_dist / 2)
rospy.sleep(1)

# Approach pose for grasping first wall
grasp_z = wall_1_z + wall_height - robot_base_height + gripper_offset - hole_offset
grasp_approach_z = grasp_z + 0.05
grasp_approach_pose = Pose(Point(wall_1_x, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))

if behavior_selection == 'using_forks':
    # Approach pose for grasping first wall
    fork_approach_pose_1 = Pose(Point(wall_1_x-0.11, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
    waypoints = []
    waypoints.append(copy.deepcopy(fork_approach_pose_1))
    (plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
    group_placer.execute(plan)
    fork_approach_pose_2 = Pose(Point(wall_1_x-0.11, wall_1_y, grasp_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
    waypoints = []
    waypoints.append(copy.deepcopy(fork_approach_pose_2))
    (plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
    group_placer.execute(plan)
    
else:
    waypoints = []
    waypoints.append(copy.deepcopy(grasp_approach_pose))
    (plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
    group_placer.execute(plan)

    # Get current pose of placer for reference
    rospy.sleep(3)
    pose_c = group_placer.get_current_pose().pose
    print "========== Current placer pose: ", pose_c
    
# Grasp pose for first wall
grasp_pose = Pose(Point(wall_1_x, wall_1_y, grasp_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
waypoints = []
waypoints.append(copy.deepcopy(grasp_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)

if behavior_selection != 'using_forks':
    # Close the gripper
    pub_gripper_p.publish(gripper_open_dist / 2 - 0.025)
    pub_gripper_d.publish(-gripper_open_dist / 2 + 0.025)
    rospy.sleep(1)

# Lift the wall
waypoints = []
waypoints.append(copy.deepcopy(grasp_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)
rospy.sleep(1)

# Linear motion in wall-plane (testing physics)
test_pose = copy.deepcopy(grasp_approach_pose)
test_pose.position.y -= 0.1
waypoints = []
waypoints.append(copy.deepcopy(test_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)
rospy.sleep(1)

# Approach pose for placing first wall
place_approach_pose = Pose(Point(place_wall_1_x, 0, grasp_approach_z), Quaternion(0, 0, -0.382683, 0.9238795))
place_approach_pose = Pose(Point(place_wall_1_x, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
waypoints = []
#waypoints.append(copy.deepcopy(pose_c))
waypoints.append(copy.deepcopy(place_approach_pose))
(plan, fraction) = group_placer.compute_cartesian_path(waypoints, 0.001, 0.0)
group_placer.execute(plan)

moveit_commander.roscpp_shutdown()
