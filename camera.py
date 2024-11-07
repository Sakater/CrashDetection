# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Initialisiere den ROS-Node
rospy.init_node('zed_depth_viewer', anonymous=True)

# Erstelle einen CvBridge, um ROS-Bilder in OpenCV-Bilder zu konvertieren
bridge = CvBridge()

# Globale Variable für das Tiefenbild
depth_image = None


# Callback für das Tiefenbild-Topic
def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")


# Subscriber für das Tiefenbild
depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, depth_callback)

# Zeige die Bilder im Stream an
while not rospy.is_shutdown():
    if depth_image is not None:
        # Optional: Tiefenbild anzeigen (konvertiere zu einem darstellbaren Format)
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)
        depth_display_colored = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

        # Zeige das Tiefenbild an
        cv2.imshow("ZED2 Depth", depth_display_colored)

        # Setze die Größe des Fensters
        cv2.resizeWindow("ZED2 Depth", 150, 120)  # Breite und Höhe in Pixeln

        # Warte auf Tastendruck, um das Bild zu schließen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Wenn ROS heruntergefahren wird, schließe die Fenster
cv2.destroyAllWindows()