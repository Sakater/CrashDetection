import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Initialisiere den ROS-Node
rospy.init_node('zed_depth_viewer', anonymous=True)

# Erstelle einen CvBridge, um ROS-Bilder in OpenCV-Bilder zu konvertieren
bridge = CvBridge()

# Globale Variable für das aktuelle Bild und die Tiefe
rgb_image = None
depth_image = None

# Callback für das RGB-Bild-Topic
def rgb_callback(msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")

# Callback für das Tiefenbild-Topic
def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

# Subscriber für RGB und Tiefenbilder
rgb_sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, rgb_callback)
depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, depth_callback)

# Funktion, um ROI auf das Bild zu zeichnen
def draw_roi(img, roi):
    # roi sollte ein Rechteck sein: (x, y, Breite, Höhe)
    x, y, w, h = roi
    # Zeichne das Rechteck auf das Bild
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

# ROI-Definition für den Fahrschlauch (du kannst diese Werte anpassen)
roi_x = 320  # ROI mittig auf der X-Achse
roi_y = 240  # Y-Position des ROIs
roi_width = 100  # Breite des ROIs
roi_height = 100  # Höhe des ROIs

# Zeige die Bilder im Stream an
while not rospy.is_shutdown():
    if rgb_image is not None and depth_image is not None:
        # Erstelle eine Kopie des RGB-Bilds für die Anzeige
        display_image = rgb_image.copy()

        # Zeichne den ROI (den Fahrschlauch) auf das Bild
        draw_roi(display_image, (roi_x, roi_y, roi_width, roi_height))

        # Zeige das Bild mit dem ROI an
        cv2.imshow("ZED2 RGB with ROI", display_image)

        # Optional: Tiefenbild anzeigen (konvertiere zu einem darstellbaren Format)
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)
        depth_display_colored = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

        # Zeige das Tiefenbild an
        cv2.imshow("ZED2 Depth", depth_display_colored)

        # Warte auf Tastendruck, um das Bild zu schließen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Wenn ROS heruntergefahren wird, schließe die Fenster
cv2.destroyAllWindows()