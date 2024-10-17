import pyzed.sl as sl
import time
import numpy as np

# Initialisiere die ZED-Kamera
zed = sl.Camera()

# Erstelle InitParameters-Objekt
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Wähle die Kameraauflösung
init_params.coordinate_units = sl.UNIT.METER  # Messeinheiten auf Meter setzen

# Öffne die Kamera
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"Fehler: {status}")
    exit(1)

# Erstelle RuntimeParameters für die Tiefenmessung
runtime_params = sl.RuntimeParameters()

# Erstelle Mat-Objekte für die Tiefendaten
depth = sl.Mat()

# Parameter für das Tracking
object_position_prev = None
object_velocity = None

try:
    while True:
        # Hole aktuelle Tiefen- und Bilddaten
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Hole die Tiefendaten
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

            # Definiere die Position des Objekts (hier z.B. Pixelkoordinate im Bild)
            x_pixel = 320
            y_pixel = 240

            # Hole die Tiefe (Entfernung zum Objekt in Metern)
            distance = depth.get_value(x_pixel, y_pixel)[1]

            if distance > 0:  # Überprüfe, ob eine valide Distanz erfasst wurde
                # Berechne die Position des Objekts in 3D (x, y, z Koordinaten)
                object_position = np.array([x_pixel, y_pixel, distance])

                # Falls eine vorherige Position existiert, berechne die Geschwindigkeit
                if object_position_prev is not None:
                    delta_position = object_position[2] - object_position_prev[2]
                    delta_time = 0.1  # Zeitintervall in Sekunden (z.B. 100ms)
                    object_velocity = delta_position / delta_time
                    print(f"Distanz: {object_position[2]} m")
                    print(f"Geschwindigkeit: {object_velocity} m/s")

                # Speichere die aktuelle Position als vorherige Position
                object_position_prev = object_position

            time.sleep(0.1)  # Warte 100ms bis zur nächsten Messung
except KeyboardInterrupt:
    print("Programm beendet.")

# Schließe die Kamera
zed.close()