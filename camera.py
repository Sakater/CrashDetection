# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Initialize the ROS node
rospy.init_node('zed_depth_viewer', anonymous=True)

# Create a CvBridge to convert ROS images to OpenCV images
bridge = CvBridge()

# Global variable for the image
image = None

# Callback for the image topic
def image_callback(msg):
    global image
    try:
        # Convert the ROS Image message to a NumPy array
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Subscriber for the image
image_sub = rospy.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image, image_callback)

# Set the window to WINDOW_NORMAL
cv2.namedWindow("ZED2 Image", cv2.WINDOW_NORMAL)

# Display the images in the stream
while not rospy.is_shutdown():
    if image is not None:
        # Display the image
        cv2.imshow("ZED2 Image", image)

        # Wait for key press to close the image
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When ROS is shut down, close the windows
cv2.destroyAllWindows()