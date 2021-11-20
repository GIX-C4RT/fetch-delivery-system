# Publish a transform from base link to markermap

# Get T base to head
tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = tf_buffer.lookup_transform("head_camera_rgb_optical_frame",
                                       "base_link", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time
pose_goal = geometry_msgs.msg.PoseStamped()
pose_goal.header.frame_id = "head_camera_rgb_optical_frame"
pose_goal.pose = box_target.box_pose
pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)

# Get T head to marker by look up "marker_map_frame", "dewey_camera" I cannot directly publish head_camera_rgb fram to marker map because it will
# causing jumps so this is only for our internal reference

# Publish the transform