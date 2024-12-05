# -*- coding: utf-8 -*-
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import numpy as np

# Brücke zwischen ROS und OpenCV
bridge = CvBridge()

# Global variables to store the steering angle, previous depth data, and image
steerings = []
steering_average = 90  # Default value for straight
steering_max_left = 180
steering_max_right = 0
max_angle = 53
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
    global previous_depth_image, previous_time

    try:
        # Konvertiere ROS Image Message in OpenCV Bild
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        current_time = time.time()

        if previous_depth_image is not None and previous_time is not None:
            # Bilde den Fahrschlauch (ROI) basierend auf dem Lenkwinkel
            image_height, image_width = depth_image.shape
            x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(angle_callback(),
                                                                         image_width, image_height, depth_image)

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
                            #print("Geschwindigkeit bei ({}, {}): {} Meter/Sekunde".format(x, y, speed))
                            if speed >= current_distance:
                                return 3
                            elif speed < current_distance and speed*1.5 >= current_distance:
                                return 2
                            elif speed*1.5 > current_distance:
                                return 1
                    #print("Momentane Distanz: {}".format(current_distance))
                    #print("Momentane Lenkung: {}".format(steering_average))

        # Speichere die aktuellen Tiefendaten und den Zeitstempel
        previous_depth_image = depth_image.copy()
        previous_time = current_time

    except Exception as e:
        rospy.logerr("Fehler beim Verarbeiten der Tiefendaten: {}".format(e))


# Callback-Funktion für das Abonnieren der Lenkungsdaten
def steering_callback(msg):
    print("hello")
    global steering_average, steerings
    if msg.data is not None:
        """steerings.append(msg.data)
        if len(steerings) > 10:
            steerings.pop(0)"""
        steering_average = msg.data #sum(steerings) / len(steerings)
        print("Updated steering_average: {}".format(steering_average))


def angle_callback():
    """Berechnet linear den Wert des Lenkwinkels im Verhältnis +-90 == 53° Lenkung
    (-) steht für Rechtslenkung, (+) für Linkslenkung"""
    #TODO: abs(average-90) verwenden anstatt max left etc????? --> schon erledigt
    global steering_average, steering_max_left, max_angle
    print("steering_average: {}, maxL: {}, maxA: {}".format(steering_average, steering_max_left, max_angle))
    vorzeichen = 1 if steering_average > 87 else -1
    angle = (max_angle / abs(steering_max_left - 87) * abs(steering_average - 87) * vorzeichen)
    print("Calculated angle: {}".format(angle))
    return angle

# Funktion zur Berechnung des Fahrschlauchs basierend auf dem Lenkwinkel und der Distanz
def calculate_roi_based_on_steering(angle, image_width, image_height, depth_image):
    # Berechne die durchschnittliche Distanz im Bild
    avg_distance = np.mean(depth_image[depth_image > 0])

    # Passe die ROI-Größe basierend auf der Distanz an
    roi_width = int(image_width / (1 + abs(angle)) * (1 / avg_distance))
    roi_height = int(image_height / 3 * (1 / avg_distance))

    x_min = int((image_width - roi_width) / 2 + angle * (image_width / 2))
    x_max = x_min + roi_width
    y_min = int(image_height / 2)
    y_max = y_min + roi_height

    # Debug-Ausgabe zur Überprüfung der ROI-Werte
    print("ROI: x_min={}, y_min={}, x_max={}, y_max={}, angle={}".format(x_min, y_min, x_max, y_max, angle))

    return x_min, y_min, x_max, y_max

def main():
    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)
    rospy.Subscriber("/ctrlcmd_steering", Int16, steering_callback)
    rospy.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image, image_callback)
    danger = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)
    #cv2.namedWindow("ZED2 Image", cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        if image is not None and previous_depth_image is not None:
            # Berechne den ROI basierend auf dem Lenkwinkel und der Distanz
            image_height, image_width, _ = image.shape
            x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(angle_callback(), image_width,
                                                                         image_height, previous_depth_image)
            print("Danger: {}".format(danger))

            # Zeichne den ROI auf das Bild
            #cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Display the image
            #cv2.imshow("ZED2 Image", image)

            # Wait for key press to close the image
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    rospy.spin()

if __name__ == "__main__":
    main()
