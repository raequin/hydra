#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def my_cartesian_move(goal_pose, move_group):
    waypoints = []
    waypoints.append(copy.deepcopy(goal_pose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
    move_group.execute(plan)

pnp_wall_1 = True
attach_wall_1 = True
pnp_wall_2 = True
attach_wall_2 = True

gripper_offset = 0.16385  # Distance from tool flange to gripping screws
gripper_zero_dist = 0.033  # Distance between gripper pad and center when closed
two_by_width = 0.0889  # 2.5 inches in m
gripper_close_dist = 0.01145  # Distance between gripper and centerline when closing on 2x4
gripper_open_padding = 0.05  #0.015  # How far extra to move gripper half when open

joint_home_placer = [0, -math.pi/2-0.2, math.pi/2, -math.pi/2, -math.pi/2, 0]
joint_home_attacher = [0, -3*math.pi/4, math.pi/2, -math.pi/2, -math.pi/2, 0]
joint_zeros = [0, 0, 0, 0, 0, 0]
sqrt_one_half = 0.707106781186547
wall_height = 0.6096
hole_offset = 0.00953  # Distance from wall top to hole center
robot_base_height = 0.1
grasp_z = wall_height - robot_base_height + gripper_offset - hole_offset + .007
grasp_approach_z = grasp_z + 0.08

nailer_offset = 0.133  # Perpendicular distance from tool flange to nailer tip

wall_1_x = 0.4
wall_1_y = -0.5

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
    rospy.sleep(1)
    
    jt = JointTrajectory()
    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ""
    jt.joint_names = ['placer_shoulder_pan_joint', 'placer_shoulder_lift_joint', 'placer_elbow_joint', 'placer_wrist_1_joint', 'placer_wrist_2_joint', 'placer_wrist_3_joint']
    p = JointTrajectoryPoint()
    p.positions = joint_home_placer
    jt.points.append(p)
    jt.points[0].time_from_start = rospy.Duration.from_sec(2)
    pub_joint_placer.publish(jt)
    
    jt = JointTrajectory()
    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ""
    jt.joint_names = ['attacher_shoulder_pan_joint', 'attacher_shoulder_lift_joint', 'attacher_elbow_joint', 'attacher_wrist_1_joint', 'attacher_wrist_2_joint', 'attacher_wrist_3_joint']
    p = JointTrajectoryPoint()
    p.positions = joint_home_attacher
    jt.points.append(p)
    jt.points[0].time_from_start = rospy.Duration.from_sec(2)    
    pub_joint_attacher.publish(jt)
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
    rospy.sleep(2)
    pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
    pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
    rospy.sleep(2)
    sys.exit(0)

# Initialize moveit_commander and rospy.
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
if sys.argv[2] == 'spawn_objects':
    # Spawn footer boards
    #model_file = open('/home/mqm/.gazebo/models/footer_long/model.sdf', 'r')
    model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/footer_simple_test/footer_long_simple.urdf.xacro', 'r')
    model_description = model_file.read()
    model_pose = Pose(Point(place_wall_1_x-.02, place_wall_1_y-.02, 0), Quaternion(0, 0, 0.3827, 0.9239))
    #model_spawner("footer_1", model_description, "", model_pose, "world")
    model_spawner_urdf("footer_1_simple", model_description, "", model_pose, "world")
    
    #model_file = open('/home/mqm/.gazebo/models/footer_short/model.sdf', 'r')
    model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/footer_simple_test/footer_simple_test.urdf.xacro', 'r')
    model_description = model_file.read()
    model_pose = Pose(Point(place_wall_2_x-.03, place_wall_2_y-.03, 0), Quaternion(0, 0, -0.3827, 0.9239))
    #model_spawner("footer_2", model_description, "", model_pose, "world")
    model_spawner_urdf("footer_2_simple", model_description, "", model_pose, "world")
    
    # Spawn walls
    if pnp_wall_1:
        model_file = open('/home/mqm/.gazebo/models/wall_plain_with_blocking/model.sdf', 'r')
        #model_file = open('/home/mqm/ROSWorkspaces/catkin_ws/src/hydra/models/wall_simple_test/wall_simple_test.urdf.xacro', 'r')
        model_description = model_file.read()
        model_pose_1 = Pose(Point(wall_1_x, wall_1_y, 0), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
        model_spawner("wall_1", model_description, "", model_pose_1, "world")
        #model_spawner_urdf("wall_1_simple", model_description, "", model_pose, "world")
        rospy.sleep(1)

        # Add wall mesh to planning scene, in hopes that it can be attached to the robot and thus ignored by self-collision detection
#        scene.add_mesh('wall_1', model_pose_1, '/home/mqm/.gazebo/models/wall_plain_with_blocking/meshes/wall_plain_with_blocking.stl')

    if pnp_wall_2:
        model_file = open('/home/mqm/.gazebo/models/wall_plain/model.sdf', 'r')
        model_description = model_file.read()
        model_pose = Pose(Point(wall_1_x-0.25, wall_1_y, 0), Quaternion(0, 0, sqrt_one_half, sqrt_one_half))
        model_spawner("wall_2", model_description, "", model_pose, "world")

#
# Pick and place wall 1
#
if pnp_wall_1:
    # Move the placer manipulator to home position
    group_placer.set_joint_value_target(joint_home_placer)
    plan = group_placer.plan()
    group_placer.execute(plan)

    # Get current pose for reference
    pose_c = group_placer.get_current_pose().pose
    print "========== Current placer pose: ", pose_c
    pose_c = group_attacher.get_current_pose().pose
    print "========== Current attacher pose: ", pose_c
    
    # Open the gripper
    pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
    pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
    rospy.sleep(1)

    # Approach pose for grasping first wall
    # Orientation is 0, pi, pi/2 = XYZ
    #grasp_approach_pose = Pose(Point(wall_1_x, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
    grasp_approach_pose = Pose(Point(wall_1_x, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, -sqrt_one_half, 0, 0))
    my_cartesian_move(grasp_approach_pose, group_placer)
    
    # Grasp pose for first wall
    grasp_pose = copy.deepcopy(grasp_approach_pose)
    grasp_pose.position.z = grasp_z - 0.05
    my_cartesian_move(grasp_pose, group_placer)
    rospy.sleep(1)
    
    # Close the gripper
    pub_gripper_p.publish(gripper_close_dist)
    pub_gripper_d.publish(-gripper_close_dist)
    rospy.sleep(1)
    
    # Lift the wall
    my_cartesian_move(grasp_approach_pose, group_placer)
    
    # Approach pose for placing first wall
    place_approach_pose = Pose(Point(place_wall_1_x+.01, place_wall_1_y, grasp_z+0.1), Quaternion(-0.3827, 0.9239, 0, 0))
    my_cartesian_move(place_approach_pose, group_placer)
    
    # Get current pose for reference
    pose_c = group_placer.get_current_pose().pose
    print "========== Current placer pose: ", pose_c
    joint_c = group_placer.get_current_joint_values()
    print "========== Current joints: ", joint_c
    
    # Place the wall
    place_pose = copy.deepcopy(place_approach_pose)
    place_pose.position.z = grasp_z + 0.0127 - 0.05
    my_cartesian_move(place_pose, group_placer)

#
# Attach wall 1
#
if attach_wall_1:
    attacher_home_pose = Pose(Point(1.3, -.2, .9), Quaternion(sqrt_one_half, sqrt_one_half, 0, 0))
    #my_cartesian_move(attacher_home_pose, group_attacher)

    w1_mid_pose  = Pose(Point(place_wall_1_x+.2, place_wall_1_y-.2, nailer_offset+.0381), Quaternion(.9239, .3827, 0, 0))
    #my_cartesian_move(w1_mid_pose, group_attacher)
    
    temp_pose = copy.deepcopy(w1_mid_pose)
    temp_pose.position.x -= .15
    temp_pose.position.y -= .15
    my_cartesian_move(temp_pose, group_attacher)

    w1_n1_approach_pose = copy.deepcopy(temp_pose)
    w1_n1_approach_pose.position.x -= .13
    w1_n1_approach_pose.position.y += .13
    my_cartesian_move(w1_n1_approach_pose, group_attacher)
    
    w1_n1_pose = copy.deepcopy(w1_n1_approach_pose)
    w1_n1_pose.position.z -= .008
    my_cartesian_move(w1_n1_pose, group_attacher)
    
    # Spawn a heavy block for holding the wall in place
    model_file = open('/home/mqm/.gazebo/models/heavy_block/model.sdf', 'r')
    model_description = model_file.read()
    model_pose = Pose(Point(place_wall_1_x, place_wall_1_y, .0292+.01905/2), Quaternion(0, 0, 0.3827, 0.9239))
    model_spawner("heavy_block_1", model_description, "", model_pose, "world")
    rospy.sleep(1)
    
    my_cartesian_move(w1_n1_approach_pose, group_attacher)
    my_cartesian_move(temp_pose, group_attacher)
    temp_pose.position.x += .27
    temp_pose.position.y += .27
    my_cartesian_move(temp_pose, group_attacher)
    
    w1_n2_approach_pose = copy.deepcopy(temp_pose)
    w1_n2_approach_pose.position.x -= .13
    w1_n2_approach_pose.position.y += .13
    my_cartesian_move(w1_n2_approach_pose, group_attacher)
    
    w1_n2_pose = copy.deepcopy(w1_n2_approach_pose)
    w1_n2_pose.position.z -= .008
    my_cartesian_move(w1_n2_pose, group_attacher)
    
    my_cartesian_move(w1_n2_approach_pose, group_attacher)
    my_cartesian_move(temp_pose, group_attacher)
    my_cartesian_move(w1_mid_pose, group_attacher)
    
    # Open the gripper
    pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
    rospy.sleep(1)
    pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)
    
    # Leave first wall
    my_cartesian_move(place_approach_pose, group_placer)
    

#
# Pick and place wall 2
#
if pnp_wall_2:
    # Approach pose for grasping second wall
    # Orientation is 0, pi, pi/2 = XYZ
    grasp_approach_pose = Pose(Point(wall_1_x-0.25, wall_1_y, grasp_approach_z), Quaternion(sqrt_one_half, -sqrt_one_half, 0, 0))
    my_cartesian_move(grasp_approach_pose, group_placer)
    
    # Grasp pose for second wall
    grasp_pose = copy.deepcopy(grasp_approach_pose)
    grasp_pose.position.z = grasp_z - 0.05
    my_cartesian_move(grasp_pose, group_placer)
    rospy.sleep(1)
    
    # Close the gripper
    pub_gripper_p.publish(gripper_close_dist)
    pub_gripper_d.publish(-gripper_close_dist)
    rospy.sleep(1)
    
    # Lift the wall
    my_cartesian_move(grasp_approach_pose, group_placer)

    temp_pose = copy.deepcopy(grasp_approach_pose)
    temp_pose.position.x += 0.7
    my_cartesian_move(temp_pose, group_placer)
    
    # Approach pose for placing second wall
    place_approach_pose = Pose(Point(place_wall_2_x-0.005, place_wall_2_y-0.035, grasp_z+0.1), Quaternion(-0.9239, 0.3827, 0, 0))
    my_cartesian_move(place_approach_pose, group_placer)
    
    # Place the wall
    place_pose = copy.deepcopy(place_approach_pose)
    place_pose.position.z = grasp_z + 0.0127 - 0.05
    my_cartesian_move(place_pose, group_placer)


#
# Attach wall 2
#
if attach_wall_2:
    #my_cartesian_move(attacher_home_pose, group_attacher)

    w2_mid_pose  = Pose(Point(place_wall_2_x+.2, place_wall_2_y+.2, nailer_offset+.0381), Quaternion(.3827, .9239, 0, 0))
    #my_cartesian_move(w2_mid_pose, group_attacher)
    
    temp_pose = copy.deepcopy(w2_mid_pose)
    temp_pose.position.x += .15
    temp_pose.position.y -= .15
    my_cartesian_move(temp_pose, group_attacher)

    w2_n1_approach_pose = copy.deepcopy(temp_pose)
    w2_n1_approach_pose.position.x -= .13
    w2_n1_approach_pose.position.y -= .13
    my_cartesian_move(w2_n1_approach_pose, group_attacher)
    
    w2_n1_pose = copy.deepcopy(w2_n1_approach_pose)
    w2_n1_pose.position.z -= .008
    my_cartesian_move(w2_n1_pose, group_attacher)
    
    # Spawn a heavy block for holding the wall in place
    model_file = open('/home/mqm/.gazebo/models/heavy_block/model.sdf', 'r')
    model_description = model_file.read()
    model_pose = Pose(Point(place_wall_2_x-.02, place_wall_2_y-.02, .0292+.01905/2), Quaternion(0, 0, -0.3827, 0.9239))
    model_spawner("heavy_block_2", model_description, "", model_pose, "world")
    
    my_cartesian_move(w2_n1_approach_pose, group_attacher)
    my_cartesian_move(temp_pose, group_attacher)
    temp_pose.position.x -= .27
    temp_pose.position.y += .27
    my_cartesian_move(temp_pose, group_attacher)
    
    w2_n2_approach_pose = copy.deepcopy(temp_pose)
    w2_n2_approach_pose.position.x -= .13
    w2_n2_approach_pose.position.y -= .13
    my_cartesian_move(w2_n2_approach_pose, group_attacher)
    
    w2_n2_pose = copy.deepcopy(w2_n2_approach_pose)
    w2_n2_pose.position.z -= .008
    my_cartesian_move(w2_n2_pose, group_attacher)
    
    my_cartesian_move(w2_n2_approach_pose, group_attacher)
    my_cartesian_move(temp_pose, group_attacher)
    my_cartesian_move(w2_mid_pose, group_attacher)
    
# Open the gripper
pub_gripper_p.publish(gripper_close_dist + gripper_open_padding)
rospy.sleep(1)
pub_gripper_d.publish(-gripper_close_dist - gripper_open_padding)

# Leave second wall
my_cartesian_move(place_approach_pose, group_placer)    

moveit_commander.roscpp_shutdown()
