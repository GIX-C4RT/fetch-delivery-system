#!/usr/bin/env python2

import sys
import copy
from math import pi
from math import sqrt
import time

import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Empty

from fetch_api import Gripper, Torso, Head, Base, Arm

from robotics_labs.msg import BoxTarget

LOWER_DEPTH = 0.27 # 27 cm
UNTUCK_SURFACE_CLEARANCE = 0.50 # 60 cm
PLACE_CLEARANCE = 0.35 # 35 cm
PLACE_POSITION_X = 0.70 # 70 cm
PLACE_POSITION_Y = 0
PLACE_ROTATION = -45 # 45 deg

class FetchController:
    '''
    A class representing the Fetch robot.
    '''
    def __init__(self):
        '''
        Instantiate an instance of the FetchController class.
        '''
        # start ROS node
        rospy.init_node('fetch_controller', anonymous=True)
        # set up publisher for goal position
        self.nav_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, latch=True, queue_size=1)
        self.initialize_grasping()

    def initialize_grasping(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm_with_torso"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper = Gripper()
        self.torso = Torso()
        self.head = Head()
        self.base = Base()
        self.arm = Arm()
        # setup clear octomap service
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    
    def pick_tray(self):
        # open gripper
        self.gripper.open()
        # get position of box
        print "waiting for box message"
        box_pose = self.get_box_pose()
        print "got box message"
        # build octomap to avoid collisions
        print "building octomap"
        self.build_octomap()
        # add common collision objects
        print "adding commmon collision objects"
        self.add_common_collision_objects()
        # move above box
        print "moving gripper above box"
        self.execute_pose_goal(box_pose)
        # move gripper around box
        print "moving gripper down to handle"
        self.change_gripper_height(-LOWER_DEPTH)
        # grab box
        print "closing gripper"
        self.gripper.close()
        # add tray as collision object
        print "adding tray collision object"
        self.add_tray_collision_object()
        # pick up tray
        print "picking up tray"
        self.change_gripper_height(LOWER_DEPTH)
        # tuck tray
        print "tucking tray"
        self.execute_tuck()
        # lower torso
        print "lowering torso"
        self.torso.set_height(0.09)
        print "done grasping"
        # remove common collision objects
        print "removing commmon collision objects"
        self.remove_common_collision_objects()
        # remove tray collision object
        print "removing tray collision object"
        self.remove_tray_collision_object()

    def place_tray(self):
        # get position of plane
        print "waiting for plane message"
        surface_height = self.get_plane_height()
        print "got surface height"
        # build octomap to avoid collisions
        print "building octomap"
        self.build_octomap_arm_extended()
        # add common collision objects
        print "adding commmon collision objects"
        self.add_common_collision_objects()
        # add tray as collision object
        print "adding tray collision object"
        self.add_tray_collision_object()
        # moving tray over plane
        print "moving tray over surface"
        self.untuck(surface_height)
        # lowering tray
        print "lowering tray"
        self.change_gripper_height(-(UNTUCK_SURFACE_CLEARANCE - PLACE_CLEARANCE))
        # opening gripper
        print "opening gripper"
        self.gripper.open()
        # remove tray collision object
        print "removing tray collision object"
        self.remove_tray_collision_object()
        # raising gripper
        print "raising gripper"
        self.change_gripper_height((UNTUCK_SURFACE_CLEARANCE))
        # remove common collision objects
        print "removing commmon collision objects"
        self.remove_common_collision_objects()
        # tucking arm
        print "tucking arm"
        self.arm.tuck()
        # wait for the arm to finish tucking
        time.sleep(5)
        # lower torso
        print "lowering torso"
        self.torso.set_height(0.09)
        

    def add_common_collision_objects(self):
        # Add base plane collision object
        # stops arm from colliding with robot base
        base_plane_pose = geometry_msgs.msg.PoseStamped()
        base_plane_pose.header.frame_id = "base_link"
        base_plane_pose.pose.position.z += 0.41
        base_plane_pose.pose.position.x += 0.17
        self.base_plane_name = "base_plane"
        rospy.sleep(2)
        self.scene.add_box(self.base_plane_name, base_plane_pose, size=(0.253, 0.57, 0.03))

        # define ground plane collision object
        ground_plane_pose = geometry_msgs.msg.PoseStamped()
        ground_plane_pose.header.frame_id = "base_link"
        self.ground_plane_name = "ground_plane"
        rospy.sleep(2)
        self.scene.add_box(self.ground_plane_name, ground_plane_pose, size=(5, 5, 0.1))

    def remove_common_collision_objects(self):
        self.scene.remove_world_object(self.ground_plane_name)
        self.scene.remove_world_object(self.base_plane_name)
    
    def get_plane_height(self):
        """
        Get plane height base frame
        """
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        plane_point = rospy.wait_for_message('plane_centroid', PointStamped, timeout=15)
        # print plane_point.header.frame_id

        transform = self.tf_buffer.lookup_transform("base_link",
                                            plane_point.header.frame_id, # source frame
                                            rospy.Time(0), # get the tf at latest available time
                                            rospy.Duration(5.0)) 

        # transform box pose from camera frame to base frame
        transformed_point = tf2_geometry_msgs.do_transform_point(plane_point, transform)

        print "height: " + str(transformed_point.point.z)
        return transformed_point.point.z

    def change_gripper_height(self, gripper_height_delta):
        # get current gripper position
        transform = self.tf_buffer.lookup_transform("base_link",
                                            "wrist_roll_link", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time

        pose = Pose()
        pose.position = transform.transform.translation
        pose.orientation = transform.transform.rotation
        # pose.position.x = transform.transform.translation.x
        # pose.position.y = transform.transform.translation.y
        # pose.position.z = transform.transform.translation.z
        # pose.orientation.x = transform.transform.rotation.x
        # pose.orientation.y = transform.transform.rotation.y
        # pose.orientation.z = transform.transform.rotation.z
        # pose.orientation.w = transform.transform.rotation.w

        waypoints = []

        # pose.position.z -= (UNTUCK_SURFACE_CLEARANCE - PLACE_CLEARANCE)
        pose.position.z += gripper_height_delta
        waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        5.0,
                                        False)         # jump_threshold

        self.move_group.execute(plan)
        # Calling `stop()` ensures that there is no residual movement
        # self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def untuck(self, height):
        # assume box is still attatched to gripper
        
        # create pose goal for gripper (in base frame)
        pose_goal = Pose()

        pose_goal.position.x = PLACE_POSITION_X
        # pose_goal.position.x = 0.7 # 70 cm
        pose_goal.position.y = PLACE_POSITION_Y
        pose_goal.position.z = height + UNTUCK_SURFACE_CLEARANCE

        # goal_orientation = tf.transformations.quaternion_from_euler(np.deg2rad(45), np.deg2rad(90), 0, axes='szyx')
        goal_orientation = tf.transformations.quaternion_from_euler(0.0, np.deg2rad(90), np.deg2rad(PLACE_ROTATION))
        # goal_orientation = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)

        pose_goal.orientation = geometry_msgs.msg.Quaternion(*goal_orientation)
        
        print pose_goal

        self.move_group.set_pose_target(pose_goal)

        # constraint to keep tray level
        target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)
        constraint = moveit_msgs.msg.Constraints()
        constraint.name = "level tray grasp constraint"
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "wrist_roll_link"
        orientation_constraint.orientation = geometry_msgs.msg.Quaternion(target_q[0],target_q[1],target_q[2],target_q[3])
        # It looks like it doesn't take values < 0.1 into account need to investigate
        # in to ompl source for more info
        orientation_constraint.absolute_x_axis_tolerance = 2 * pi # let tray spin horizontally
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1
        constraint.orientation_constraints.append(orientation_constraint)
        self.move_group.set_path_constraints(constraint)

        # move_group.set_start_state(move_group.get_current_state())

        # plan trajectory
        self.move_group.set_planning_time(15)
        myplan = self.move_group.plan()

        if not myplan.joint_trajectory.points:  # True if trajectory contains points
            exit()
        
        # execute trajectory
        self.move_group.execute(myplan)
        print "finished untuck"

        # Calling `stop()` ensures that there is no residual movement
        print "stopping untuck movement"
        # self.move_group.stop()

    def dock(self):
        # Docking to table
        pass

    def navigate_to(self, x, y, theta):
        # Send a navigating goal
        NavGoal = PoseStamped()
        NavGoal.header.frame_id = "map"
        NavGoal.header.stamp = rospy.Time.now()
        position = Point(x=x, y=y, z=0)
        orientation = quaternion_from_euler(0, 0, theta)
        NavGoal.pose = Pose(position, orientation)
        self.nav_goal_pub.publish(NavGoal)

    def get_box_pose(self):
        """
        Get box pose in base frame
        """
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform = self.tf_buffer.lookup_transform("base_link",
                                            "head_camera_rgb_optical_frame", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time

        while True:
            box_target1 = rospy.wait_for_message('box_target', BoxTarget, timeout=15)
            box_target2 = rospy.wait_for_message('box_target', BoxTarget, timeout=15)
            dist_metric = sqrt((box_target1.box_pose.position.x - box_target2.box_pose.position.x)**2 + 
                                    (box_target1.box_pose.position.y - box_target2.box_pose.position.y)**2 + 
                                    (box_target1.box_pose.position.z - box_target2.box_pose.position.z)**2)
            scale_metric = abs(box_target1.box_scale.scale.x * box_target1.box_scale.scale.y * box_target1.box_scale.scale.z -
                                box_target2.box_scale.scale.x * box_target2.box_scale.scale.y * box_target2.box_scale.scale.z)

            # print "dist_metric = ", dist_metric, ";  scale_metric = ", scale_metric
            if dist_metric < 0.02 and scale_metric < 0.02:
                box_target = box_target2
                break

        # transform box pose from camera frame to base frame
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "head_camera_rgb_optical_frame"
        pose_goal.pose = box_target.box_pose
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)

        pose_transformed.pose.position.z += 0.5
        # print pose_transformed.pose.position.x
        # print pose_transformed.pose.position.y
        # print pose_transformed.pose.position.z

        angles = euler_from_quaternion([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
        target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, angles[2])

        # if the box maker has scale in y larger than scale in x then we need to turn the gripper 90 degrees
        if box_target.box_scale.scale.y > box_target.box_scale.scale.x:
            target_q = tf.transformations.quaternion_multiply(target_q, tf.transformations.quaternion_from_euler(3.14 / 2.0, 0.0, 0.0))

        pose_transformed.pose.orientation.x = target_q[0]
        pose_transformed.pose.orientation.y = target_q[1]
        pose_transformed.pose.orientation.z = target_q[2]
        pose_transformed.pose.orientation.w = target_q[3]

        return pose_transformed

    def build_octomap(self):
        # clear any pre-existing octomap
        self.clear_octomap()
        # wait for octomap to clear
        time.sleep(1)
        # scan area to build octomap
        self.head.pan_tilt(0.0, 0.0)
        self.head.pan_tilt(-1.5, 0.8)
        self.head.pan_tilt(1.5, 0.8)
        self.head.pan_tilt(-1.5, 0.4)
        self.head.pan_tilt(1.5, 0.4)
        self.head.pan_tilt(-1.5, 0.0)
        self.head.pan_tilt(1.5, 0.0)
        self.head.pan_tilt(0.0, 0.0)

    def build_octomap_arm_extended(self):
        # clear any pre-existing octomap
        self.clear_octomap()
        # wait for octomap to clear
        time.sleep(1)
        # scan area to build octomap
        self.head.pan_tilt(0.0, 0.0)
        # self.head.pan_tilt(-1.5, 0.8)
        self.head.pan_tilt(1.5, 0.8)
        # self.head.pan_tilt(-1.5, 0.4)
        self.head.pan_tilt(1.5, 0.4)
        self.head.pan_tilt(-1.5, 0.0)
        self.head.pan_tilt(1.5, 0.0)
        self.head.pan_tilt(0.0, 0.0)
        print "octomap built"

    def execute_pose_goal(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.set_planning_time(15)
        myplan = self.move_group.plan()
        if not myplan.joint_trajectory.points:  # True if trajectory contains points
            exit()

        self.move_group.execute(myplan)
        # Calling `stop()` ensures that there is no residual movement
        # self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def add_tray_collision_object(self):
        self.tray_name = "tray"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "wrist_roll_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x += 0.325
        rospy.sleep(2)
        self.scene.add_box(self.tray_name, box_pose, size=(0.1, 0.2, 0.35))

        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        touch_links.append("wrist_roll_link")
        self.scene.attach_box(self.move_group.get_end_effector_link(), self.tray_name, touch_links=touch_links)

    def remove_tray_collision_object(self):
        # detatch collision box from gripper
        self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name=self.tray_name)
        # remove collision box from scene
        self.scene.remove_world_object(self.tray_name)

    def execute_tuck(self):

        self.head.pan_tilt(-1.5, 0.0)
        self.head.pan_tilt(1.5, 0.0)
        self.head.pan_tilt(0.0, 0.0)

        target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)
        joint_goal = [0 for i in range(8)]
        joint_goal[0] = 0.35#0.0857168138027
        joint_goal[1] = -1.58481834789
        joint_goal[2] = 0.433445118112
        joint_goal[3] = -1.53769913621
        joint_goal[4] = 1.80451970107
        joint_goal[5] = 2.0000173681
        joint_goal[6] = 1.63692856729
        joint_goal[7] = 2.86215073034

        self.move_group.set_joint_value_target(joint_goal)

        constraint = moveit_msgs.msg.Constraints()
        constraint.name = "dewey grasp constraint"
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "wrist_roll_link"
        orientation_constraint.orientation = geometry_msgs.msg.Quaternion(target_q[0],target_q[1],target_q[2],target_q[3])
        # It looks like it didn't took value < 0.1 into account need to investigate
        # in to ompl source for more info
        orientation_constraint.absolute_x_axis_tolerance = 2 * pi
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1
        constraint.orientation_constraints.append(orientation_constraint)
        self.move_group.set_path_constraints(constraint)

        # move_group.set_start_state(move_group.get_current_state())

        self.move_group.set_planning_time(15)
        myplan = self.move_group.plan()

        if not myplan.joint_trajectory.points:  # True if trajectory contains points
            exit()

        self.move_group.execute(myplan)
        print "finished tuck"
        # Calling `stop()` ensures that there is no residual movement
        print "stopping tuck movement"
        # self.move_group.stop()

if __name__ == "__main__":
    myf = FetchController()
    myf.torso.set_height(0.09)
    myf.head.pan_tilt(0.0, 0.8)
    myf.pick_tray()
    # myf.navigate_to(None)
    myf.head.pan_tilt(0.0, 0.8)
    myf.place_tray()
    # rospy.spin()