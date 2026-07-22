#!/usr/bin/env python3
"""Ruta de inspeccion por waypoints: navegacion + foto + analisis + pausa.

Evolucion de ruta_waypoints.py: en lugar de delegar la mision completa en
followWaypoints(), se navega waypoint a waypoint con goToPose(). Al llegar a
cada punto, el robot captura una imagen de la camara (topic /camera/image_raw),
la guarda en disco con el nombre del waypoint, la analiza al instante con
scripts/vision/inspeccionar.py (sin esperar a que termine la mision), y espera
PAUSA_SEG segundos antes de continuar. Al finalizar la mision se imprime y
guarda el mismo informe de 9 puntos que generaria inspeccionar.py a mano.

Uso:
    python3 scripts/ruta_waypoints_foto.py

Requisitos (una sola vez, van a MANUAL.md):
    sudo apt install ros-jazzy-nav2-simple-commander ros-jazzy-cv-bridge python3-opencv
"""

import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2                                    # OpenCV: escritura de imagenes (y futura vision).
import rclpy
from cv_bridge import CvBridge                # Conversor mensaje ROS Image <-> matriz OpenCV.
from geometry_msgs.msg import PoseStamped
from rclpy.wait_for_message import wait_for_message  # Espera bloqueante de UN mensaje.
from sensor_msgs.msg import Image
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# scripts/vision/ vive junto a este script: se añade al path para poder
# reusar el despachador y sus detectores sin duplicar codigo.
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision"))
from inspeccionar import inspeccionar_foto, ORDEN_RUTA, SIN_VISION, DIR_ANALIZADAS  # noqa: E402

# ---------------------------------------------------------------------------
# CONFIGURACION
# ---------------------------------------------------------------------------

ORIENTAR_HACIA_SIGUIENTE = False               # Flag para reorientar el robot hacia el siguiente waypoint.
PAUSA_SEG = 7.0                               # Parada total en cada waypoint (s).
INTENTOS_POR_WAYPOINT = 2                     # Reintentos ante fallo transitorio del BT
                                              # (p. ej. timeout de acuse del planner bajo carga).
TOPIC_CAMARA = "/camera/image_raw"            # Camara del Waffle (via bridge de Gazebo).
DIR_FOTOS = os.path.expanduser("~/TFG/fotos_waypoints")  # Carpeta de salida.

# (nombre, x, y, qz, qw) en frame 'map'. Ultimo punto: retorno a base.
WAYPOINTS = [
    ("Cajas Para Entrega (Pasillo 1)",     5.860,  -2.820, -0.043, 0.999),
    ("Pasillo 1 - Estanterias Carritos",   9.853, -14.882, -0.001, 1.000),
    ("Caja Grande Pasillo 2",              2.728, -11.665, -0.613, 0.790),
    ("Pasillo 3 - carretilla",            -3.225, -20.157,  0.441, 0.897),
    ("Esquina Derecha Pasillo 5",        -11.322, -20.789, -0.839, 0.545),
    ("Pasillo 5 - armario central",      -11.985,  -9.304,  0.896, -0.444),
    ("Pasillo 4 - viga centro mapa",      -7.534,  -2.010, -0.698, 0.716),
    ("Estanterias centrales dcha - viga", -8.500,   2.261, -0.450, 0.893),
    ("Vuelta a base (spawn)",              0.000,   0.000,  0.000, 1.000),
]

# ---------------------------------------------------------------------------
# UTILIDADES
# ---------------------------------------------------------------------------

def yaw_a_quaternion(yaw: float) -> tuple[float, float]:
    """(qz, qw) de una rotacion plana: qz=sin(yaw/2), qw=cos(yaw/2)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def crear_pose(navigator: BasicNavigator,
               x: float, y: float, qz: float, qw: float) -> PoseStamped:
    """PoseStamped en frame 'map' con timestamp del reloj de simulacion."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def construir_ruta(navigator: BasicNavigator) -> list[PoseStamped]:
    """Poses de la ruta, reorientadas al rumbo del tramo si procede."""
    poses = []
    n = len(WAYPOINTS)
    for i, (nombre, x, y, qz, qw) in enumerate(WAYPOINTS):
        if ORIENTAR_HACIA_SIGUIENTE and i < n - 1:
            _, x_sig, y_sig, _, _ = WAYPOINTS[i + 1]
            yaw = math.atan2(y_sig - y, x_sig - x)
            qz, qw = yaw_a_quaternion(yaw)
        poses.append(crear_pose(navigator, x, y, qz, qw))
    return poses


def tomar_foto(navigator: BasicNavigator, bridge: CvBridge,
               indice: int, nombre: str) -> str | None:
    """Captura UNA imagen de la camara, la guarda en DIR_FOTOS y devuelve
    la ruta guardada (None si no se ha podido capturar).

    wait_for_message crea una suscripcion temporal, espera el primer
    mensaje del topic y la destruye: exactamente lo que necesitamos para
    una foto puntual sin mantener una suscripcion permanente.
    """
    ok, msg = wait_for_message(Image, navigator, TOPIC_CAMARA, time_to_wait=5.0)
    if not ok:
        navigator.get_logger().warn(
            f"Sin imagen de {TOPIC_CAMARA} en 5 s; se omite la foto de '{nombre}'."
        )
        return None

    # Conversion del mensaje ROS a matriz OpenCV en BGR (formato nativo de cv2):
    imagen = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Nombre de archivo: indice + nombre saneado + marca temporal.
    seguro = nombre.lower().replace(" ", "_").replace("-", "").replace("__", "_")
    marca = datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta = os.path.join(DIR_FOTOS, f"wp{indice + 1:02d}_{seguro}_{marca}.png")
    cv2.imwrite(ruta, imagen)
    navigator.get_logger().info(f"Foto guardada: {ruta}")
    return ruta


# ---------------------------------------------------------------------------
# PROGRAMA PRINCIPAL
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()
    bridge = CvBridge()
    os.makedirs(DIR_FOTOS, exist_ok=True)     # Crea la carpeta de fotos si no existe.

    navigator.get_logger().info("Esperando a que Nav2 este activo...")
    navigator.waitUntilNav2Active(localizer="amcl")

    ruta = construir_ruta(navigator)
    navigator.get_logger().info(f"Mision de inspeccion: {len(ruta)} waypoints, "
                                f"pausa de {PAUSA_SEG:.0f} s con foto+analisis en cada uno.")
    t_inicio = time.monotonic()               # Cronometro de la mision (metrica para la memoria).
    estados = {}                              # wp -> mensaje de inspeccion, para el informe final

    for i, pose in enumerate(ruta):
        nombre = WAYPOINTS[i][0]
        alcanzado = False

        # Reintento por waypoint: un fallo del BT puede ser transitorio (timeout
        # de acuse del action server bajo carga de CPU) con el robot ya en el
        # objetivo o muy cerca. El segundo intento suele triunfar de inmediato.
        for intento in range(1, INTENTOS_POR_WAYPOINT + 1):
            sufijo = "" if intento == 1 else f" (reintento {intento - 1})"
            navigator.get_logger().info(
                f"-> Navegando a {i + 1}/{len(ruta)}: {nombre}{sufijo}")
            # Refrescar el timestamp: la pose se creo al inicio de la mision y
            # su stamp queda obsoleto para goals lanzados minutos despues.
            pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(pose)          # Un goal individual: el script recupera
                                              # el control al completarse.
            while not navigator.isTaskComplete():
                pass                          # (aqui podria leerse feedback.distance_remaining)

            resultado = navigator.getResult()
            if resultado == TaskResult.SUCCEEDED:
                alcanzado = True
                break
            navigator.get_logger().warn(
                f"Fallo el intento {intento}/{INTENTOS_POR_WAYPOINT} hacia "
                f"'{nombre}' ({resultado}).")

        if not alcanzado:
            # Si un waypoint falla tras agotar los reintentos, se registra y se
            # continua con el siguiente: criterio de mision robusta (un fallo
            # puntual no aborta la patrulla).
            navigator.get_logger().error(f"No se alcanzo '{nombre}'. "
                                         "Se continua con el siguiente waypoint.")
            continue

        # Waypoint alcanzado: foto + analisis + resto de la pausa hasta PAUSA_SEG.
        t_foto = time.monotonic()
        ruta_foto = tomar_foto(navigator, bridge, i, nombre)
        wp_id = f"wp{i + 1:02d}"
        if ruta_foto is not None:
            try:
                _, msg, _ = inspeccionar_foto(ruta_foto)   # guarda tambien la figura anotada
                if msg:
                    estados[wp_id] = msg
                    navigator.get_logger().info(f"   Analisis: {msg}")
            except Exception as e:
                # Un fallo del analisis (p. ej. falta la referencia de un wp)
                # NO debe abortar la mision de navegacion: se registra y se
                # sigue. La foto ya esta guardada y puede reanalizarse luego
                # a mano con inspeccionar.py.
                navigator.get_logger().warn(f"   Analisis fallido en {wp_id}: {e}")
        transcurrido = time.monotonic() - t_foto
        if transcurrido < PAUSA_SEG:
            time.sleep(PAUSA_SEG - transcurrido)

    duracion = time.monotonic() - t_inicio
    navigator.get_logger().info(f"Mision completada en {duracion:.1f} s "
                                f"({duracion / 60:.1f} min).")

    # --- Informe de inspeccion: mismo formato que generaria inspeccionar.py
    # a mano sobre el directorio, pero ya con los resultados calculados
    # durante la mision (no hace falta releer ni reanalizar nada). ---
    lineas = [f"INFORME DE INSPECCION - {datetime.now().strftime('%d/%m/%Y %H:%M')}",
              f"Mision: {DIR_FOTOS}", ""]
    for wp in ORDEN_RUTA:
        estado = estados.get(wp) or SIN_VISION.get(wp) or "sin foto en esta mision"
        lineas.append(f"  {wp}  {estado}")
    avisos = sum(1 for wp in ORDEN_RUTA
                 if (estados.get(wp) or "").startswith(("AVISO", "NO VALIDABLE")))
    lineas += ["", f"Puntos con aviso o no validables: {avisos}"]
    informe = "\n".join(lineas)
    destino_dir = Path(DIR_FOTOS) / DIR_ANALIZADAS
    destino_dir.mkdir(parents=True, exist_ok=True)
    destino = destino_dir / f"informe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    destino.write_text(informe + "\n", encoding="utf-8")
    navigator.get_logger().info("\n" + informe + f"\n\nInforme guardado en: {destino}")

    # NO llamar a navigator.lifecycleShutdown(): apagaria todos los nodos de
    # Nav2 (AMCL incluido) e impediria relanzar misiones sin reiniciar el
    # stack. El flujo A->B->C exige que Nav2 sobreviva al script: solo se
    # cierra el contexto ROS de ESTE proceso.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
