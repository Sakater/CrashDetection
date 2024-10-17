# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import time

# Konstanten
MAX_STEERING_ANGLE = 30.0  # Maximaler Lenkwinkel in Grad
MAX_STEERING_VALUE = 100.0  # Maximale Lenkwert (absolut)
prev_distance = None
prev_time = None
bridge = CvBridge()
steering_angle = 0.0  # Initialisiere den Lenkwinkel
current_speed = 0.0  # Aktuelle Geschwindigkeit


# Funktion zur Berechnung des Lenkwinkels
def calculate_steering_angle(steering_value):
    return (steering_value / MAX_STEERING_VALUE) * MAX_STEERING_ANGLE


# Callback für den Lenkwert
def steering_callback(msg):
    global steering_angle
    steering_angle = calculate_steering_angle(msg.data)
    rospy.loginfo("Lenkwinkel: {:.2f} Grad".format(steering_angle))


# Funktion, um den Fahrschlauch zu zeichnen
def draw_fahrschlauch(angle, image_width, image_height):
    center_x = image_width // 2
    y_min = image_height // 2  # Fahrschlauch beginnt in der Mitte
    y_max = image_height  # Fahrschlauch endet unten

    offset = int(angle / MAX_STEERING_ANGLE * center_x)

    return (center_x - offset, center_x + offset, y_min, y_max)


# Funktion zur Distanzberechnung
def calculate_distance_in_roi(depth_image, roi):
    x_min, x_max, y_min, y_max = roi
    roi_depth = depth_image[y_min:y_max, x_min:x_max]

    # Finde den minimalen Wert (nächster Punkt) innerhalb des Fahrschlauchs
    min_distance = np.nanmin(roi_depth)
    return min_distance


# Geschwindigkeit basierend auf Distanzänderung berechnen
def calculate_speed(current_distance):
    global prev_distance, prev_time, current_speed

    current_time = time.time()

    if prev_distance is None or prev_time is None:
        prev_distance = current_distance
        prev_time = current_time
        return 0.0  # Initial keine Geschwindigkeit

    delta_distance = prev_distance - current_distance
    delta_time = current_time - prev_time

    # Geschwindigkeit = Distanzänderung / Zeit
    if delta_time > 0:
        current_speed = delta_distance / delta_time

    # Aktualisiere vorherige Werte
    prev_distance = current_distance
    prev_time = current_time

    return current_speed


# Callback für Tiefenbild
def depth_callback(msg):
    global steering_angle

    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        rospy.logerr("Failed to convert depth image: {}".format(e))
        return

    depth_image = np.nan_to_num(depth_image, nan=np.inf)  # Entferne NaN-Werte

    # Bildgröße
    height, width = depth_image.shape

    # Bestimme den ROI
    roi = draw_fahrschlauch(steering_angle, width, height)

    # Berechne die Distanz zum nächsten Objekt im Fahrschlauch
    min_distance = calculate_distance_in_roi(depth_image, roi)
    rospy.loginfo("Nächste Distanz im Fahrschlauch: {:.2f} m".format(min_distance))

    # Berechne die aktuelle Geschwindigkeit
    speed = calculate_speed(min_distance)
    rospy.loginfo("Aktuelle Geschwindigkeit: {:.2f} m/s".format(speed))

    # Print the distance and speed
    print("Distanz: {:.2f} m".format(min_distance))
    print("Geschwindigkeit: {:.2f} m/s".format(speed))


def main():
    rospy.init_node('zed_fahrschlauch_viewer', anonymous=True)

    # Abonniere das Steuerungs-Topic für den Lenkwinkel
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)

    # Abonniere das Tiefenbild-Topic der ZED-Kamera
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)

    # ROS am Laufen halten
    rospy.spin()


if __name__ == "__main__":
    main()