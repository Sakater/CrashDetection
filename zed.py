# -*- coding: utf-8 -*-
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Constants
MAX_STEERING_ANGLE = 30.0
MAX_STEERING_VALUE = 100.0

# Initialize CvBridge
bridge = CvBridge()

# Function to calculate the ROI based on the steering angle
def calculate_roi_based_on_steering(angle, image_width, image_height):
    roi_width = int(image_width / (1 + abs(angle) / 10))
    roi_height = int(image_height / 3)

    x_min = int((image_width - roi_width) / 2)
    x_max = x_min + roi_width
    y_min = int(image_height / 2)
    y_max = y_min + roi_height

    return x_min, y_min, x_max, y_max

# Function to calculate the steering angle
def calculate_steering_angle(steering_value):
    return (steering_value / MAX_STEERING_VALUE) * MAX_STEERING_ANGLE

# Function to calculate speed from depth image
def calculate_speed_from_depth(depth_image):
    # Example calculation: average depth value in the image
    avg_depth = cv2.mean(depth_image)[0]
    speed = avg_depth / 10.0  # Example conversion factor
    return speed

# Function to calculate the distance to the nearest object
def calculate_distance_to_nearest_object(depth_image):
    # Ensure the depth image is single-channel
    if len(depth_image.shape) > 2:
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    min_depth = cv2.minMaxLoc(depth_image)[0]
    return min_depth

# Callback function for the steering value
def steering_callback(msg):
    global steering_angle
    steering_value = msg.data
    steering_angle = calculate_steering_angle(steering_value)
    rospy.loginfo("Lenkwinkel: {:.2f} Grad".format(steering_angle))

# Function to draw the driving lane and ROI on the image
def draw_fahrschlauch_and_roi(image, angle, image_width, image_height):
    center_x = image_width // 2
    y_min = image_height // 2
    y_max = image_height

    offset = int(angle / MAX_STEERING_ANGLE * center_x)

    # Draw the driving lane
    cv2.line(image, (center_x - offset, y_min), (center_x - offset, y_max), (0, 255, 0), 3)
    cv2.line(image, (center_x + offset, y_min), (center_x + offset, y_max), (0, 255, 0), 3)

    # Calculate and draw the ROI
    x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(angle, image_width, image_height)
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

    return image

# Callback function for the image
def image_callback(msg):
    global steering_angle

    try:
        # Check the encoding of the image
        if msg.encoding == "32FC1":
            frame = bridge.imgmsg_to_cv2(msg, "32FC1")
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            frame = cv2.cvtColor(frame.astype('uint8'), cv2.COLOR_GRAY2BGR)
        else:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        height, width, _ = frame.shape

        frame_with_fahrschlauch_and_roi = draw_fahrschlauch_and_roi(frame, steering_angle, width, height)

        # Calculate speed from depth image
        speed = calculate_speed_from_depth(frame)
        rospy.loginfo("Geschwindigkeit: {:.2f} m/s".format(speed))

        # Calculate distance to the nearest object
        distance = calculate_distance_to_nearest_object(frame)
        rospy.loginfo("Distanz zum nächsten Objekt: {:.2f} m".format(distance))

        cv2.imshow("ZED Camera with Fahrschlauch and ROI", frame_with_fahrschlauch_and_roi)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {}".format(e))

def main():
    global steering_angle
    steering_angle = 0

    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)

    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, image_callback)
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)

    rospy.spin()

if __name__ == "__main__":
    main()