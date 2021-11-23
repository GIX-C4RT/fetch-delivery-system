import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import sqrt
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from robotics_labs.msg import BoxTarget
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time
from fetch_api import Gripper
from fetch_api import Torso
from fetch_api import Head
from fetch_api import Base
from std_srvs.srv import Empty
TablePoses = (Pose(position=Point(x = 6.24092817307,y = 5.13014507294, z = 0), orientation=Quaternion(x = 0, y = 0, z = 0.68882361899, w = 0.724928977156)), 
 Pose(position=Point(x = 5.93524551392, y = 1.50320911407, z = 0), orientation=Quaternion(x = 0, y = 0, z = 0.684483803425, w = 0.729028067257)))
class FetchController:
    def __init__(self):
        rospy.init_node('fetch_controller', anonymous=True)
        self.nav_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, latch=True, queue_size=1)
        
    def init_grasping(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm_with_torso"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper = Gripper()
        self.torso = Torso()
        self.head = Head()
        self.base = Base()
        # Clearning any pre-existing octomap
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        # Add ground plane collision object
        base_plane_pose = geometry_msgs.msg.PoseStamped()
        base_plane_pose.header.frame_id = "base_link"
        base_plane_pose.pose.position.z += 0.41
        base_plane_pose.pose.position.x += 0.17
        self.base_plane_name = "base_plane"
        rospy.sleep(2)
        self.scene.add_box(self.base_plane_name, base_plane_pose, size=(0.253, 0.57, 0.03))

    def grasp_item(self):
        # Execute moveit final script
        self.init_grasping()
        box_pose = self.get_box_pose()
        # if(box_pose.pose.position.x > 0.5):
        #     box_pose.pose.position.x
        #     self.base.go_forward(box_pose.pose.position.x - 0.5)
        #     box_pose = self.get_box_pose()
        self.build_octomap()
        self.execute_pose_goal(box_pose)
        self.execute_approach()
        self.execute_retract()
        self.base.go_forward(-0.5)
        self.execute_tuck()
        self.torso.set_height(0.09)

    def dock(self):
        # Docking to table
        pass

    def navigate_to(self, target_pose):
        # Send a navigating goal
        NavGoal = PoseStamped()
        NavGoal.header.frame_id = "map"
        NavGoal.header.stamp = rospy.Time.now()
        # NavGoal.pose = target_pose
        NavGoal.pose = TablePoses[0]
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

            print "dist_metric = ", dist_metric, ";  scale_metric = ", scale_metric
            if dist_metric < 0.02 and scale_metric < 0.02:
                box_target = box_target2
                break

        # transform box pose from camera frame to base frame
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "head_camera_rgb_optical_frame"
        pose_goal.pose = box_target.box_pose
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)

        pose_transformed.pose.position.z += 0.5
        print pose_transformed.pose.position.x
        print pose_transformed.pose.position.y
        print pose_transformed.pose.position.z

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
        self.head.pan_tilt(0.0, 0.0)
        self.head.pan_tilt(-1.5, 0.8)
        self.head.pan_tilt(1.5, 0.8)
        self.head.pan_tilt(-1.5, 0.4)
        self.head.pan_tilt(1.5, 0.4)
        self.head.pan_tilt(-1.5, 0.0)
        self.head.pan_tilt(1.5, 0.0)
        self.head.pan_tilt(0.0, 0.0)

    def execute_pose_goal(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.set_planning_time(15)
        myplan = self.move_group.plan()
        if not myplan.joint_trajectory.points:  # True if trajectory contains points
            exit()

        self.move_group.execute(myplan)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def execute_approach(self):
        transform = self.tf_buffer.lookup_transform("base_link",
                                       "wrist_roll_link", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w

        waypoints = []
        pose.position.z -= 0.1
        waypoints.append(copy.deepcopy(pose))

        pose.position.z -= 0.13
        waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        5.0,         # jump_threshold
                                        False)         

        self.move_group.execute(plan)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def execute_retract(self):
        self.gripper.close()
        transform = self.tf_buffer.lookup_transform("base_link",
                                            "wrist_roll_link", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w

        waypoints = []
        pose.position.z += 0.1
        waypoints.append(copy.deepcopy(pose))

        pose.position.z += 0.15
        waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        5.0,
                                        False)         # jump_threshold

        self.move_group.execute(plan)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def execute_tuck(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "wrist_roll_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x += 0.325
        box_name = "box"
        rospy.sleep(2)
        self.scene.add_box(box_name, box_pose, size=(0.35, 0.2, 0.35))

        ground_plane_pose = geometry_msgs.msg.PoseStamped()
        ground_plane_pose.header.frame_id = "base_link"
        ground_plane_name = "ground_plane"
        rospy.sleep(2)
        self.scene.add_box(ground_plane_name, ground_plane_pose, size=(5, 5, 0.1))

        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        touch_links.append("wrist_roll_link")
        self.scene.attach_box(self.move_group.get_end_effector_link(), box_name, touch_links=touch_links)

        self.clear_octomap()

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
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name=box_name)
        self.scene.remove_world_object(box_name)
        self.scene.remove_world_object(ground_plane_name)
        self.scene.remove_world_object(self.base_plane_name)

if __name__ == "__main__":
    myf = FetchController()
    myf.grasp_item()
    myf.navigate_to(None)
    rospy.spin()