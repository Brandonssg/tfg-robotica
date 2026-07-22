#!/usr/bin/env python3
"""
scripts/detector_obstaculos.py
Nodo independiente de percepción activada por eventos (Fase 4): vigila
/scan durante toda la misión y, cuando varios puntos del LiDAR caen sobre
celdas que el mapa estático marca como LIBRES, interpreta que hay un
obstáculo dinámico no contemplado. En ese instante:
  1) toma una foto con el último frame de cámara disponible (evidencia;
     limitado por el alcance/FOV de la cámara si el obstáculo está lejos
     - limitación ya documentada en el punto 6 de vuestros pendientes),
  2) registra el evento (timestamp, pose del robot, nº de puntos
     anómalos, ruta de la foto) en fotos_obstaculos/log_obstaculos.csv.

Deliberadamente NO interviene en la navegación: Nav2 ya replanifica solo
porque su costmap se alimenta directamente de /scan. Este nodo es un
observador aparte sin ningún topic de salida que la navegación consuma
- si fallase, la misión sigue intacta (mismo principio de separación
navegación-crítica / percepción-analítica que el try/except del punto 27).

Requiere (una sola vez, va a MANUAL.md):
    pip install pillow pyyaml --break-system-packages

Uso (tercer terminal, en paralelo a Nav2 y a la misión):
    python3 scripts/detector_obstaculos.py
"""
import csv
import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Image as ImageMsg
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException

# ---------------------------------------------------------------------------
# CONFIGURACION
# ---------------------------------------------------------------------------
MAPA_YAML = os.path.expanduser("~/TFG/maps/mapa_tfg_semi_3_limpio.yaml")
DIR_SALIDA = os.path.expanduser("~/TFG/fotos_obstaculos")
LOG_CSV = os.path.join(DIR_SALIDA, "log_obstaculos.csv")

TOPIC_SCAN = "/scan"
TOPIC_CAMARA = "/camera/image_raw"
FRAME_MAPA = "map"
FRAME_ROBOT = "base_footprint"

ALCANCE_MAX_CHEQUEO = 4.0      # m. Puntos mas lejanos no se comprueban:
                                # mayor error de pixel acumulado y menos
                                # relevantes para un obstaculo inmediato.
MIN_PUNTOS_ANOMALOS = 150        # puntos anomalos en UN scan para considerarlo
                                # señal real y no ruido de un punto suelto.
SCANS_CONSECUTIVOS = 3         # scans seguidos con anomalia antes de confirmar.
COOLDOWN_SEG = 15.0             # tras un evento, ignora nuevas confirmaciones
                                # este tiempo (evita loguear el mismo
                                # obstaculo estatico repetidamente).
VALOR_LIBRE = 254              # convencion trinaria del mapa (ver limpiar_mapas.py)


# ---------------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------------
def yaw_de_cuaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def cargar_mapa(ruta_yaml):
    """Carga el mapa trinario (.pgm) y sus metadatos (.yaml), mismo formato
    que consume nav2_map_server. Devuelve la matriz numpy, la resolucion
    (m/celda) y el origen [x, y] del mapa en el frame 'map'."""
    with open(ruta_yaml, "r") as f:
        meta = yaml.safe_load(f)

    ruta_pgm = meta["image"]
    if not os.path.isabs(ruta_pgm):
        ruta_pgm = os.path.join(os.path.dirname(ruta_yaml), ruta_pgm)

    img = PILImage.open(ruta_pgm).convert("L")
    matriz = np.array(img)
    resolucion = float(meta["resolution"])
    origen_x, origen_y = float(meta["origin"][0]), float(meta["origin"][1])
    return matriz, resolucion, origen_x, origen_y


class DetectorObstaculos(Node):
    def __init__(self):
        super().__init__("detector_obstaculos")

        self.mapa, self.resolucion, self.origen_x, self.origen_y = cargar_mapa(MAPA_YAML)
        self.alto_px, self.ancho_px = self.mapa.shape
        self.get_logger().info(
            f"Mapa cargado: {self.ancho_px}x{self.alto_px} px, "
            f"resolucion {self.resolucion} m/celda, origen ({self.origen_x}, {self.origen_y})"
        )

        os.makedirs(DIR_SALIDA, exist_ok=True)
        self._inicializar_log()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.ultima_imagen = None  # ultimo frame de camara, actualizado en cada mensaje

        self.contador_confirmacion = 0
        self.en_cooldown_hasta = 0.0  # segundos (reloj de sistema), 0 = sin cooldown activo

        self.create_subscription(ImageMsg, TOPIC_CAMARA, self._callback_imagen, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self._callback_scan, 10)

        self.get_logger().info("Detector de obstaculos dinamicos activo.")

    # -----------------------------------------------------------------
    def _inicializar_log(self):
        nuevo = not os.path.exists(LOG_CSV)
        self._log_f = open(LOG_CSV, "a", newline="")
        self._log_w = csv.writer(self._log_f)
        if nuevo:
            self._log_w.writerow(["timestamp", "x", "y", "puntos_anomalos", "foto"])
            self._log_f.flush()

    def _callback_imagen(self, msg):
        self.ultima_imagen = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # -----------------------------------------------------------------
    def _pose_robot(self):
        """Pose actual del robot en el frame 'map', o None si TF no esta lista."""
        try:
            tf = self.tf_buffer.lookup_transform(FRAME_MAPA, FRAME_ROBOT, Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _callback_scan(self, msg):
        ahora = self.get_clock().now().nanoseconds / 1e9
        if ahora < self.en_cooldown_hasta:
            return  # evento reciente: no seguir comprobando hasta que pase el cooldown

        try:
            tf = self.tf_buffer.lookup_transform(FRAME_MAPA, msg.header.frame_id, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return  # TF aun no disponible (arranque, o AMCL sin pose inicial)

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = yaw_de_cuaternion(tf.transform.rotation)

        rangos = np.array(msg.ranges)
        angulos = msg.angle_min + np.arange(len(rangos)) * msg.angle_increment

        validos = np.isfinite(rangos) & (rangos >= msg.range_min) & \
            (rangos <= min(msg.range_max, ALCANCE_MAX_CHEQUEO))
        if not np.any(validos):
            self.contador_confirmacion = 0
            return

        r = rangos[validos]
        a = angulos[validos]

        # Punto del scan (frame del sensor) -> frame 'map': rotar por el yaw
        # del robot y trasladar por su posicion. LiDAR 2D, sin componente Z.
        x_local = r * np.cos(a)
        y_local = r * np.sin(a)
        x_mapa = tx + x_local * math.cos(yaw) - y_local * math.sin(yaw)
        y_mapa = ty + x_local * math.sin(yaw) + y_local * math.cos(yaw)

        # (x, y) del mundo -> indices de pixel. Fila 0 del .pgm = arriba =
        # y maxima; por eso se invierte con (alto_px - 1 - fila_desde_abajo).
        col = ((x_mapa - self.origen_x) / self.resolucion).astype(int)
        fila_desde_abajo = ((y_mapa - self.origen_y) / self.resolucion).astype(int)
        fila = self.alto_px - 1 - fila_desde_abajo

        dentro = (fila >= 0) & (fila < self.alto_px) & (col >= 0) & (col < self.ancho_px)
        fila, col = fila[dentro], col[dentro]
        if len(fila) == 0:
            self.contador_confirmacion = 0
            return

        valores_mapa = self.mapa[fila, col]
        n_anomalos = int(np.sum(valores_mapa == VALOR_LIBRE))

        if n_anomalos >= MIN_PUNTOS_ANOMALOS:
            self.contador_confirmacion += 1
        else:
            self.contador_confirmacion = 0

        if self.contador_confirmacion >= SCANS_CONSECUTIVOS:
            self._registrar_evento(n_anomalos)
            self.contador_confirmacion = 0
            self.en_cooldown_hasta = ahora + COOLDOWN_SEG

    # -----------------------------------------------------------------
    def _registrar_evento(self, n_anomalos):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        pose = self._pose_robot()
        x, y = pose if pose else (float("nan"), float("nan"))

        ruta_foto = ""
        if self.ultima_imagen is not None:
            ruta_foto = os.path.join(DIR_SALIDA, f"obstaculo_{ts}.png")
            cv2.imwrite(ruta_foto, self.ultima_imagen)
        else:
            self.get_logger().warn("Obstaculo detectado pero aun no hay frame de camara disponible.")

        self._log_w.writerow([ts, round(x, 3), round(y, 3), n_anomalos, ruta_foto])
        self._log_f.flush()

        self.get_logger().info(
            f"OBSTACULO DINAMICO detectado en ({x:.2f}, {y:.2f}) — "
            f"{n_anomalos} puntos anomalos. Foto: {ruta_foto or '(no disponible)'}"
        )

    def destroy_node(self):
        self._log_f.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = DetectorObstaculos()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()