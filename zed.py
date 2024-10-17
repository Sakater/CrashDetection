import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from zed_interfaces.msg import ObjectsStamped
import cv2
import numpy as np
from cv_bridge import CvBridge




# Funktion zur Berechnung des Fahrschlauchs basierend auf dem Lenkwinkel
def calculate_roi_based_on_steering(angle, image_width, image_height):
    roi_width = int(image_width / (1 + abs(angle) / 10))  # Verengung des Fahrschlauchs basierend auf dem Lenkwinkel
    roi_height = int(image_height / 3)  # Begrenze die Höhe des ROI

    x_min = int((image_width - roi_width) / 2)
    x_max = x_min + roi_width
    y_min = int(image_height / 2)  # Beginne in der Mitte des Bildes
    y_max = y_min + roi_height

    return x_min, y_min, x_max, y_max


# Funktion zur Berechnung des Lenkwinkels
def calculate_steering_angle(steering_value):
    # Berechne den relativen Winkel basierend auf dem Maximalwert
    return (steering_value / MAX_STEERING_VALUE) * MAX_STEERING_ANGLE


# Callback-Funktion für das Abonnieren des Lenkungswerts
def steering_callback(msg):
    steering_value = msg.data  # Der absolute Steuerwert
    steering_angle = calculate_steering_angle(steering_value)
    rospy.loginfo(f"Lenkwinkel: {steering_angle:.2f} Grad")

    # Hier kannst du den berechneten Winkel verwenden, um den Fahrschlauch anzupassen
    # z.B.:
    # x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(steering_angle, image_width, image_height)
    # print(f"Fahrschlauch für Winkel {steering_angle} Grad: {x_min}, {y_min}, {x_max}, {y_max}")


# Funktion, um den Fahrschlauch basierend auf dem Lenkwinkel zu zeichnen
def draw_fahrschlauch(image, angle, image_width, image_height):
    # Definiere den ROI (Fahrschlauch) basierend auf dem Lenkwinkel
    center_x = image_width // 2
    y_min = image_height // 2  # Beginn des Fahrschlauchs
    y_max = image_height  # Ende des Fahrschlauchs

    # Berechne die Verschiebung auf der X-Achse basierend auf dem Lenkwinkel
    offset = int(angle / MAX_STEERING_ANGLE * center_x)

    # Zeichne den Fahrschlauch
    cv2.line(image, (center_x - offset, y_min), (center_x - offset, y_max), (0, 255, 0), 3)
    cv2.line(image, (center_x + offset, y_min), (center_x + offset, y_max), (0, 255, 0), 3)
    return image


# Callback-Funktion für das Bild
def image_callback(msg):
    global steering_angle

    # Konvertiere ROS-Bild zu OpenCV-Bild
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Bildgröße
    height, width, _ = frame.shape

    # Fahrschlauch auf dem Bild einzeichnen
    frame_with_fahrschlauch = draw_fahrschlauch(frame, steering_angle, width, height)

    # Zeige das Bild mit OpenCV an
    cv2.imshow("ZED Camera with Fahrschlauch", frame_with_fahrschlauch)
    cv2.waitKey(1)


# Callback-Funktion für das Abonnieren der Tiefendaten
def depth_callback(msg):
    try:
        # Konvertiere ROS Image Message in OpenCV Bild
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Bilde den Fahrschlauch (ROI) basierend auf dem Lenkwinkel
        steering_angle = 15  # Beispielwert für Lenkwinkel (kann dynamisch angepasst werden)
        image_height, image_width = depth_image.shape
        x_min, y_min, x_max, y_max = calculate_roi_based_on_steering(steering_angle, image_width, image_height)

        # Gehe durch den definierten ROI-Bereich (Fahrschlauch)
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                distance = depth_image[y, x]  # Hole die Tiefendistanz an der Pixelposition
                if distance > 0:  # Nur valide Distanzen
                    print(f"Objekt bei ({x}, {y}) hat eine Entfernung von: {distance} Meter")

    except Exception as e:
        rospy.logerr(f"Fehler beim Verarbeiten der Tiefendaten: {e}")


# Callback für Objekt-Detektion (optional, wenn du auch Hindernisse detektieren möchtest)
def object_callback(msg):
    for obj in msg.objects:
        print(f"Objekt {obj.label} in Reichweite von {obj.position[0]} Meter.")


def main():
    # Maximaler Einschlagwinkel der Räder (in Grad)
    MAX_STEERING_ANGLE = 30.0  # Beispiel: ±30 Grad

    # Maximaler absoluter Steuerwert (abhängig von deinem Fahrzeug)
    MAX_STEERING_VALUE = 100.0  # Beispiel: ±100
    # Brücke zwischen ROS und OpenCV
    bridge = CvBridge()
    # Initialisiere den ROS-Knoten
    rospy.init_node('zed_fahrschlauch_analyse', anonymous=True)

    # Abonniere das ROS-Topic für die Tiefendaten
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)

    # Optional: Abonniere das ROS-Topic für Objekt-Detektion
    rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, object_callback)

    # Initialisiere den ROS-Knoten
    rospy.init_node('steering_angle_calculator', anonymous=True)

    # Abonniere das ROS-Topic für den Steuerwert
    rospy.Subscriber("/ctrlcmd_steering", Float32, steering_callback)

    # Abonniere das ZED-Kamera-Topic
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, image_callback)

    # rospy.Publisher("/zed2/zed_node/velocity", Float32)

    # ROS-Node am Laufen halten
    rospy.spin()


if __name__ == "__main__":
    main()
