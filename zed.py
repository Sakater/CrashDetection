# -*- coding: utf-8 -*-
import time

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Brücke zwischen ROS und OpenCV
bridge = CvBridge()

# Global variables to store the steering angle and previous depth data
steering_angles = []
steering_angle_average = 87  # Default value for straight
steering_max_left = 130
steering_max_right = 40
previous_depth_image = None
previous_time = None


# Funktion zur Berechnung des Fahrschlauchs basierend auf dem Lenkwinkel
def calculate_roi_based_on_steering(angle, image_width, image_height):
    roi_width = int(image_width / (1 + abs(angle - 90) / 10))
    roi_height = int(image_height / 3)
    x_min = int((image_width - roi_width) / 2)
    x_max = x_min + roi_width
    y_min = int(image_height / 2)
    y_max = y_min + roi_height
    return x_min, y_min, x_max, y_max


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


def main():
    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
