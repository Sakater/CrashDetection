# -*- coding: utf-8 -*-
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np

# Brücke zwischen ROS und OpenCV
bridge = CvBridge()

# Global variables to store the steering angle, previous depth data, and image
steering_angles = []
steering_angle_average = 87  # Default value for straight
steering_max_left = 180
steering_max_right = 0
angle = 53
previous_depth_image = None
previous_time = None
image = None

# Callback-Funktion für das Abonnieren der Bilddaten
def image_callback(msg):
    global image
    try:
        # Convert the ROS Image message to a NumPy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    except Exception as e:
        rospy.logerr("Error: {0}".format(e))

# Callback-Funktion für das Abonnieren der Tiefendaten
def depth_callback(msg):
    global previous_depth_image, previous_time, steering_angle_average

    try:
        # Konvertiere ROS Image Message in OpenCV Bild
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        current_time = time.time()

        if previous_depth_image is not None and previous_time is not None:
            # Bilde den Fahrschlauch (ROI) basierend auf dem Lenkwinkel
            image_height, image_width = depth_image.shape
            x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(steering_angle_average,
                                                                         image_width, image_height)

            # Berechne die Geschwindigkeit
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    current_distance = depth_image[y, x]
                    previous_distance = previous_depth_image[y, x]
                    if current_distance > 0 and previous_distance > 0:
                        distance_change = current_distance - previous_distance
                        time_change = current_time - previous_time
                        if time_change > 0:
                            speed = 0 if abs(distance_change / time_change) < 1 else abs(distance_change / time_change)
                            print("Geschwindigkeit bei ({}, {}): {} Meter/Sekunde".format(x, y, speed))

                    print("Momentane Distanz: {}".format(current_distance))

        # Speichere die aktuellen Tiefendaten und den Zeitstempel
        previous_depth_image = depth_image.copy()
        previous_time = current_time

    except Exception as e:
        rospy.logerr("Fehler beim Verarbeiten der Tiefendaten: {}".format(e))

# Callback-Funktion für das Abonnieren der Lenkungsdaten
def steering_callback(msg):
    global steering_angle_average, steering_angles
    steering_angles.append(msg.data)
    if len(steering_angles) > 10:
        steering_angles.pop(0)
    steering_angle_average = sum(steering_angles) / len(steering_angles)

# Funktion zur Berechnung des Fahrschlauchs basierend auf dem Lenkwinkel
def calculate_roi_based_on_steering(angle, image_width, image_height):
    roi_width = int(image_width / (1 + abs(angle - 90) / 10))
    roi_height = int(image_height / 3)
    x_min = int((image_width - roi_width) / 2)
    x_max = x_min + roi_width
    y_min = int(image_height / 2)
    y_max = y_min + roi_height
    return x_min, y_min, x_max, y_max

def main():
    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)
    rospy.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image, image_callback)

    cv2.namedWindow("ZED2 Image", cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        if image is not None:
            # Berechne den ROI basierend auf dem Lenkwinkel
            image_height, image_width, _ = image.shape
            x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(steering_angle_average, image_width, image_height)

            # Zeichne den ROI auf das Bild
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Display the image
            cv2.imshow("ZED2 Image", image)

            # Wait for key press to close the image
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    rospy.spin()

if __name__ == "__main__":
    main()