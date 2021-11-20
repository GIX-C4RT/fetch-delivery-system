import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
count = 0
current_img = None
def callback(msg):
	global current_img
	current_img = msg

rospy.init_node("dewey_photographer_node")
rospy.Subscriber("/head_camera/rgb/image_raw", Image, callback)

while not rospy.is_shutdown():
	input()
	print "taking", count, "th image"
	count += 1
	cv_image = bridge.imgmsg_to_cv2(current_img, desired_encoding='bgr8')
	cv2.imwrite(str(count) + ".jpg", cv_image)
