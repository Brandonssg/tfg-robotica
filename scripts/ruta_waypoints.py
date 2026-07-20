#!/usr/bin/env python3
"""Ruta predefinida por waypoints para el TurtleBot3 en el almacén (TFG).

Recorre de forma autónoma la secuencia de waypoints capturados con
`tf2_echo map base_footprint`, usando la API nav2_simple_commander sobre
la pila Nav2 ya lanzada (navigation2.launch.py con el mapa limpio y
nav2_params_warehouse.yaml).

Esta ruta es el banco de pruebas del caso de uso:
  - Fase 3 (linea base): recorrido completo sin obstaculos.
  - Fase 4 (experimento): mismo recorrido con obstaculo dinamico
    insertado en Gazebo -> deteccion -> replanificacion.

Uso:
    python3 scripts/ruta_waypoints.py

Requisito (una sola vez, va a MANUAL.md):
    sudo apt install ros-jazzy-nav2-simple-commander
"""

import math

import rclpy                                  # Cliente Python de ROS2.
from geometry_msgs.msg import PoseStamped     # Mensaje de pose con frame y timestamp.
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,                           # API de alto nivel sobre las acciones de Nav2.
    TaskResult,                               # Enumerado del resultado (SUCCEEDED/CANCELED/FAILED).
)

# ---------------------------------------------------------------------------
# CONFIGURACION
# ---------------------------------------------------------------------------

# Si True, la orientacion de cada waypoint se recalcula para apuntar hacia el
# waypoint siguiente (rumbo del tramo), y solo el ultimo conserva la
# orientacion capturada. Evita que el robot gire sobre si mismo en cada parada
# para clavar la orientacion que tenia el operador al capturar la pose.
# Si False, se usan las orientaciones capturadas tal cual.
ORIENTAR_HACIA_SIGUIENTE = True

# Waypoints capturados con `ros2 run tf2_ros tf2_echo map base_footprint`.
# Formato: (nombre, x, y, qz, qw) en el frame 'map'.
# El nombre documenta el punto (y aparece en el log durante la ejecucion).
WAYPOINTS = [
    ("Viga Pasillo Izquierda (P1)",        7.363,  -5.556, -0.697, 0.717),
    ("Pasillo 1 - Estanterias Carritos",  11.287, -14.933,  0.026, 1.000),
    ("Caja Grande Pasillo 2",              2.919, -19.779,  0.496, 0.868),
    ("Pasillo 3 - vacio",                 -2.598, -11.172,  0.915, 0.404),
    ("Esquina Derecha Pasillo 5",        -12.720, -23.098, -0.827, 0.562),
    ("Pasillo 5 - armario central",      -12.960, -10.945,  0.921, -0.390),
    ("Pasillo 4 - viga centro mapa",      -7.915,  -5.029, -0.602, 0.799),
    ("Estanterias centrales dcha - viga", -7.521,   1.236, -0.472, 0.882),
    ("Vuelta a base (spawn)",              0.000,   0.000,  0.000, 1.000),
]

# ---------------------------------------------------------------------------
# UTILIDADES
# ---------------------------------------------------------------------------

def yaw_a_quaternion(yaw: float) -> tuple[float, float]:
    """Convierte un angulo de guiñada (rad) a los componentes (qz, qw).

    Para rotaciones puras en el plano XY, el quaternion se reduce a
    qz = sin(yaw/2), qw = cos(yaw/2) (qx = qy = 0). Es la misma
    convencion que imprime tf2_echo.
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def crear_pose(navigator: BasicNavigator,
               x: float, y: float, qz: float, qw: float) -> PoseStamped:
    """Construye un PoseStamped en el frame 'map' listo para Nav2."""
    pose = PoseStamped()
    pose.header.frame_id = "map"                       # Coordenadas absolutas del mapa.
    pose.header.stamp = navigator.get_clock().now().to_msg()  # Timestamp actual (reloj de sim).
    pose.pose.position.x = x
    pose.pose.position.y = y
    # Orientacion como quaternion plano (solo rotacion en Z):
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def construir_ruta(navigator: BasicNavigator) -> list[PoseStamped]:
    """Convierte la lista WAYPOINTS en poses, reorientando si procede."""
    poses = []
    n = len(WAYPOINTS)
    for i, (nombre, x, y, qz, qw) in enumerate(WAYPOINTS):
        if ORIENTAR_HACIA_SIGUIENTE and i < n - 1:
            # Rumbo geometrico hacia el siguiente waypoint:
            _, x_sig, y_sig, _, _ = WAYPOINTS[i + 1]
            yaw = math.atan2(y_sig - y, x_sig - x)     # Angulo del vector (dx, dy).
            qz, qw = yaw_a_quaternion(yaw)
        poses.append(crear_pose(navigator, x, y, qz, qw))
    return poses


# ---------------------------------------------------------------------------
# PROGRAMA PRINCIPAL
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()                                       # Arranca el contexto ROS2 del proceso.
    navigator = BasicNavigator()                       # Nodo cliente de las acciones de Nav2.

    # Espera a que la pila Nav2 este completamente activa (equivale a ver
    # "Managed nodes are active" en el log). 'amcl' indica que la
    # localizacion corre con AMCL; como la pose inicial ya se fija por
    # parametro (set_initial_pose en nav2_params_warehouse.yaml), no hace
    # falta llamar a setInitialPose() desde aqui.
    navigator.get_logger().info("Esperando a que Nav2 este activo...")
    navigator.waitUntilNav2Active(localizer="amcl")

    ruta = construir_ruta(navigator)
    navigator.get_logger().info(
        f"Iniciando ruta de {len(ruta)} waypoints "
        f"(reorientacion automatica: {ORIENTAR_HACIA_SIGUIENTE})"
    )

    navigator.followWaypoints(ruta)                    # Lanza la mision completa.

    # Bucle de supervision: mientras la mision no termine, Nav2 publica
    # feedback con el indice del waypoint en curso. Lo usamos para loguear
    # el progreso con los nombres descriptivos.
    ultimo_wp = -1
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and feedback.current_waypoint != ultimo_wp:
            ultimo_wp = feedback.current_waypoint
            nombre = WAYPOINTS[ultimo_wp][0]
            navigator.get_logger().info(
                f"-> Waypoint {ultimo_wp + 1}/{len(ruta)}: {nombre}"
            )

    # Resultado final de la mision:
    resultado = navigator.getResult()
    if resultado == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Ruta completada con exito.")
    elif resultado == TaskResult.CANCELED:
        navigator.get_logger().warn("Ruta cancelada.")
    elif resultado == TaskResult.FAILED:
        navigator.get_logger().error("La ruta ha fallado.")

    navigator.lifecycleShutdown()                      # Apagado ordenado del cliente.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
