# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge


# Funktion zur Berechnung des Fahrschlauchs basierend auf dem Lenkwinkel
def calculate_roi_based_on_steering(angle, image_width, image_height):
    roi_width = int(image_width / (1 + abs(angle) / 10))
    roi_height = int(image_height / 3)

    x_min = int((image_width - roi_width) / 2)
    x_max = x_min + roi_width
    y_min = int(image_height / 2)
    y_max = y_min + roi_height

    return x_min, y_min, x_max, y_max


# Funktion zur Berechnung des Lenkwinkels
def calculate_steering_angle(steering_value):
    return (steering_value / MAX_STEERING_VALUE) * MAX_STEERING_ANGLE


# Callback-Funktion für das Abonnieren des Lenkungswerts
def steering_callback(msg):
    steering_value = msg.data
    steering_angle = calculate_steering_angle(steering_value)
    rospy.loginfo("Lenkwinkel: {:.2f} Grad".format(steering_angle))


# Funktion, um den Fahrschlauch basierend auf dem Lenkwinkel zu zeichnen
def draw_fahrschlauch(image, angle, image_width, image_height):
    center_x = image_width // 2
    y_min = image_height // 2
    y_max = image_height

    offset = int(angle / MAX_STEERING_ANGLE * center_x)

    cv2.line(image, (center_x - offset, y_min), (center_x - offset, y_max), (0, 255, 0), 3)
    cv2.line(image, (center_x + offset, y_min), (center_x + offset, y_max), (0, 255, 0), 3)
    return image


# Callback-Funktion für das Bild
def image_callback(msg):
    global steering_angle

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    height, width, _ = frame.shape

    frame_with_fahrschlauch = draw_fahrschlauch(frame, steering_angle, width, height)

    cv2.imshow("ZED Camera with Fahrschlauch", frame_with_fahrschlauch)
    cv2.waitKey(1)


# Callback-Funktion für das Abonnieren der Tiefendaten
def depth_callback(msg):
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        steering_angle = 15
        image_height, image_width = depth_image.shape
        x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(steering_angle, image_width, image_height)

        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                distance = depth_image[y, x]
                if distance > 0:
                    print("Objekt bei ({}, {}) hat eine Entfernung von: {} Meter".format(x, y, distance))

    except Exception as e:
        rospy.logerr("Fehler beim Verarbeiten der Tiefendaten: {}".format(e))


def main():
    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)

    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, image_callback)

    rospy.spin()


if __name__ == "__main__":
    MAX_STEERING_ANGLE = 30.0
    MAX_STEERING_VALUE = 100.0
    bridge = CvBridge()
    main()