# Manual de instalación y uso

## Requisitos
- Ubuntu 24.04 LTS

## Instalación de ROS2 Jazzy
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

## Instalación de Gazebo Harmonic
sudo apt install -y ros-jazzy-ros-gz

## Herramientas de desarrollo
sudo apt install -y python3-colcon-common-extensions python3-rosdep git
sudo rosdep init
rosdep update

## Instalación de TurtleBot3
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-simulations
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

## Dependencias de navegación por waypoints y captura de imágenes

Necesarias para los scripts de ruta predefinida y misión de inspección
(`scripts/ruta_waypoints.py` y `scripts/ruta_waypoints_foto.py`):

```
sudo apt install -y ros-jazzy-nav2-simple-commander ros-jazzy-cv-bridge python3-opencv
```

- `nav2-simple-commander`: API Python de alto nivel sobre las acciones de Nav2.
- `cv-bridge`: conversión entre mensajes ROS `sensor_msgs/Image` y matrices OpenCV.
- `python3-opencv`: OpenCV (escritura de imágenes y procesado de visión).

## Uso
### Lanzar simulación base para pruebas con TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Control manual por teclado (terminal aparte)
ros2 run turtlebot3_teleop teleop_keyboard


## Entorno final: almacén (warehouse)

Como entorno a usar para este proyecto y más avanzado al `turtlebot3_world` de fábrica, se ha preparado un almacén industrial
(modelo Fuel de OpenRobotics/MovAi), del que se ha eliminado el robot original (Tugbot) para
sustituirlo por el TurtleBot3 Waffle. El mundo está en `worlds/warehouse_turtlebot3.sdf` y el
launch file que lo arranca y spawnea el robot, en `launch/warehouse_tb3.launch.py`.

```
ros2 launch launch/warehouse_tb3.launch.py
```

**Nota**: en el primer arranque con conexión a internet, Gazebo descarga
automáticamente los modelos de Fuel (estanterías, carros, almacén) a la caché
local `~/.gz/fuel/`. Los arranques posteriores no requieren conexión.

**Nota conocida**: el TurtleBot3 se renderiza en gris plano en este mundo (incompatibilidad
conocida entre las mallas `.dae` del paquete `turtlebot3_gazebo` y el motor de render Ogre2 de
Gazebo Harmonic). No afecta a sensores, TF ni navegación — verificado mediante los topics del
bridge (`/odom`, `/scan`, `/imu`, `/tf`, `/camera/camera_info`, `/cmd_vel`).



## SLAM: construcción del mapa

Con la simulación base lanzada (`warehouse_tb3.launch.py`), en otra terminal:

```
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$HOME/TFG/config/slam_params_warehouse.yaml
```

El archivo `config/slam_params_warehouse.yaml` contiene la configuración afinada
para este entorno. Los cambios clave respecto a los valores por defecto:


- `minimum_travel_distance/heading: 0.3` — procesa scans con más frecuencia,
  mejorando la cobertura en giros.

En caso de no conseguir un mapeado correcto puedes probar con los siguientes cambios:
- `do_loop_closing: false` — en el almacén los pasillos de estanterías son
  geométricamente muy similares entre sí, lo que provocaba falsos cierres de
  bucle (el scan-matcher "reconocía" un pasillo equivocado y deformaba el mapa).

- `distance_variance_penalty: 2.0` y `angle_variance_penalty: 2.0` — penalizan
  correcciones grandes de pose, evitando saltos bruscos del mapa.

- `correlation_search_space_dimension: 0.3` (por defecto 0.5) — restringe la
  búsqueda del scan-matcher local para que confíe más en la odometría.

Mueve el robot con teleop para cubrir todo el mapa, a velocidad moderada y
evitando giros bruscos sobre sí mismo. Cuando esté completo, guárdalo:

```
ros2 run nav2_map_server map_saver_cli -f ~/TFG/maps/mapa_tfg_semi_3
```

Esto genera `mapa_tfg_semi_3.pgm` (imagen del mapa) y `mapa_tfg_semi_3.yaml`
(metadatos: resolución 0.05 m/celda y origen). El mapa validado para navegación
es `maps/mapa_tfg_semi_3`; los demás archivos de `maps/` son iteraciones
previas del proceso de ajuste, conservadas como evidencia.


## Postprocesado del mapa

El mapa crudo de SLAM Toolbox contiene valores de gris intermedios (bordes
difusos, celdas semi-observadas). El script `scripts/limpiar_mapas.py` lo
reasigna a los tres valores trinarios que espera Nav2 (0 ocupado, 205
desconocido, 254 libre), con criterio conservador: los valores intermedios
caen a desconocido, nunca a libre, para no crear zonas transitables falsas.

```
python3 scripts/limpiar_mapas.py maps/mapa_tfg_semi_3.pgm maps/mapa_tfg_semi_3_limpio.pgm
```
 
El `.yaml` del mapa limpio es una copia del original apuntando al nuevo `.pgm`.
**El mapa validado para navegación es `maps/mapa_tfg_semi_3_limpio`.**


## Navegación autónoma (Nav2)

Con la simulación base lanzada (`warehouse_tb3.launch.py`) y ya cargada del
todo (el almacén renderizado y el robot respondiendo), en otra terminal:

```
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$HOME/TFG/maps/mapa_tfg_semi_3_limpio.yaml \
  params_file:=$HOME/TFG/config/nav2_params_warehouse.yaml
```

`config/nav2_params_warehouse.yaml` parte del `waffle.yaml` del paquete
`turtlebot3_navigation2`, con este cambio clave:

- `set_initial_pose: true` con pose `(0, 0, yaw 0)` — AMCL publica la
  transform `map→odom` desde el arranque, sin esperar a un "2D Pose
  Estimate" manual en RViz. Sin esto, el costmap global no puede activarse
  (necesita esa transform), el lifecycle manager agota su timeout y deja
  `bt_navigator` y el resto de nodos en estado `inactive`: los goals se
  rechazan con "Action server is inactive".
  **Si se cambia el punto de spawn del robot en el launch, hay que
  actualizar esta pose (en coordenadas del frame `map`).**

### Verificación de arranque correcto

En el log deben aparecer, para AMBOS managers:

```
[lifecycle_manager_localization]: Managed nodes are active
[lifecycle_manager_navigation]: Managed nodes are active
```

Solo entonces el sistema acepta goals. Uso: botón "Nav2 Goal" en RViz →
clic en destino → arrastrar para fijar orientación. El robot planifica la
ruta global (línea sobre el mapa) y la sigue esquivando obstáculos con el
costmap local.

### Recuperación sin relanzar

Si la pila quedó en `inactive` (goals rechazados), se puede reintentar la
secuencia de activación sin cerrar nada:

```
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Comprobar el estado de un nodo concreto: `ros2 lifecycle get /bt_navigator`
(debe devolver `active`).

### Notas de log (benignas)

- `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`: restos de
  memoria compartida DDS de sesiones anteriores; Fast DDS usa UDP local
  como alternativa. Sin efecto.
- `incompatible QoS (/particle_cloud)`: discrepancia de QoS entre RViz y
  AMCL en la visualización de partículas. Sin efecto en la navegación.
- `Message Filter dropping message` en RViz durante los primeros segundos:
  desaparecen en cuanto AMCL publica `map→odom`.


## Limpieza de emergencia

`scripts/reinicio_simuladores.sh` mata todos los procesos de simulación
(`pkill -9`) y reinicia el daemon de ROS2. **Es el botón de emergencia, no
el procedimiento estándar**: el cierre normal sigue siendo Ctrl+C en orden
inverso al lanzamiento (teleop → RViz/Nav2 → SLAM → Gazebo), esperando el
prompt en cada paso. Usar el script solo cuando queden procesos zombi
(síntoma típico: dos publishers en `ros2 topic info /clock --verbose`). 


## Ruta predefinida (waypoints) y misión de inspección

Con la simulación y Nav2 lanzados (ver sección anterior) y el arranque
verificado ("Managed nodes are active" en ambos lifecycle managers), la ruta
del caso de uso se ejecuta con uno de estos dos scripts:

**Ruta simple** (recorre los waypoints sin paradas):

```
python3 scripts/ruta_waypoints.py
```

**Misión de inspección** (en cada waypoint: parada de 7 s y captura de una
foto de la cámara):

```
python3 scripts/ruta_waypoints_foto.py
```

Ambos esperan automáticamente a que Nav2 esté activo, por lo que pueden
lanzarse inmediatamente después del launch. La ruta consta de 8 waypoints
con nombre descriptivo más un noveno de retorno a la base (0, 0), formando
un lazo cerrado: al terminar, el robot queda en su estado inicial y la
misión puede relanzarse sin recolocar nada.

Salida esperada: el log del script anuncia cada waypoint al alcanzarlo
(`-> Waypoint N/9: <nombre>`), y al finalizar imprime el resultado y la
duración total de la misión. Si un waypoint no puede alcanzarse, se
registra el error y la misión continúa con el siguiente.

Las fotos se guardan en `~/TFG/fotos_waypoints/` con el formato
`wpNN_<nombre>_<fecha>_<hora>.png`. Este directorio contiene datos
generados y está excluido del repositorio (`.gitignore`).

La pausa de 7 s por waypoint simula el tiempo de inspección del punto de
interés y garantiza que la captura se realiza con el robot completamente
detenido. Las fotos son la entrada del análisis de visión artificial
(detección de obstáculos, Fase 4).

**Configuración de encuadre**: en el script, el flag
`ORIENTAR_HACIA_SIGUIENTE` controla la orientación en cada waypoint. Con
`False` (valor en uso), el robot adopta la orientación capturada
manualmente en cada punto — el yaw del waypoint define el encuadre de la
foto. Con `True`, se reorienta automáticamente hacia el waypoint
siguiente (navegación más fluida, sin control del encuadre). Para
modificar la ruta basta editar la lista `WAYPOINTS` del script; las poses
se capturan con `ros2 run tf2_ros tf2_echo map base_footprint`.

## Escenarios de inspección: creación, carga y eventos dinámicos

Esta sección documenta el flujo para crear y reproducir escenarios de
inspección (Fase 4), y el mecanismo de inyección de eventos dinámicos
usado para la demo de replanificación de Nav2.

### 1. Concepto: mundo base, escenario y evento

- **Mundo base** (`worlds/warehouse_expand.sdf`): el almacén y todo el
  mobiliario/soporte *permanente*. Se mapea una única vez con SLAM Toolbox
  y da lugar al mapa de navegación (`maps/mapa_inspeccion_limpio`).
- **Escenario** (`escenarios/escenario_*.yaml`): el estado inicial de las
  señales *mutables* de cada punto de inspección (conos, toolboxes,
  personal, etc.). No forma parte del mundo base ni del mapa: se
  inyecta en tiempo de ejecución sobre el mundo ya cargado.
- **Evento** (`eventos/*.yaml`): una inyección que ocurre *a mitad de
  misión*, no al inicio. Es el mecanismo usado para el obstáculo dinámico
  que dispara la replanificación de Nav2.

La separación escenario/evento no es solo organizativa: si un elemento de
inspección se tratara como si fuera un obstáculo inesperado, el detector
de obstáculo dinámico (que compara el escaneo LiDAR contra el mapa)
generaría falsos positivos en cada punto de inspección. Los escenarios
existen para evitar justamente eso.

### 2. Cómo generar un nuevo escenario desde Gazebo

1. Lanza el mundo base y coloca los elementos deseados con
   `Resource Spawner → Fuel Resources` (busca el modelo por nombre, en  
   https://app.gazebosim.org/OpenRobotics encontrarás algunos ejemplos),
   ajustando su posición y orientación con el editor de transformar.
2. Guarda el mundo con los elementos colocados como una **plantilla
   separada** (`File → Save World As` → `worlds/world_escenario2.sdf`).
   No sobrescribas nunca `warehouse_expand.sdf`: la plantilla es
   desechable, el mundo base no.
3. Abre la plantilla guardada y localiza cada elemento añadido buscando
   `<include>` (los del mundo original ya estaban antes; los nuevos son
   los que tú acabas de colocar). Cada bloque trae:

   ```xml
   <include>
     <uri>file:///home/bran/.gz/fuel/fuel.gazebosim.org/openrobotics/models/toolbox/2</uri>
     <name>Toolbox_1</name>
     <pose>-3.907 0.01 0.0 0.0 0.0 0.0</pose>
   </include>
   ```

4. Traslada `uri` + `/model.sdf`, `name` y `pose` a un YAML de escenario
   con el formato:

   ```yaml
   wp08_toolbox1:
     modelo: "/home/bran/.gz/fuel/fuel.gazebosim.org/openrobotics/models/toolbox/2/model.sdf"
     pose: [-3.907, 0.01, 0.0, 0.0, 0.0, 0.0]
   ```

   La clave (`wp08_toolbox1`) es el nombre de instancia que se usará al
   spawnear y eliminar el elemento; no tiene por qué coincidir con el
   `<name>` autogenerado por Gazebo.
   
   Recomiendo el uso de IA para agilizar este último paso.

Ver `worlds/world_escenario2.sdf` para una plantilla en blanco con esta
misma guía incrustada, pensada para crear escenarios adicionales sin
partir de cero.


### 3. Servicios de Gazebo para crear/eliminar elementos en caliente

Con el mundo ya corriendo (Gazebo, Nav2 y AMCL activos), es posible
añadir y quitar modelos sin reiniciar la simulación ni perder la
localización del robot.

Primero, averigua el nombre del mundo activo:

```bash
gz service -l | grep create
```

Spawnear un elemento:

```bash
gz service -s /world/NOMBRE_MUNDO/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/bran/.gz/fuel/.../toolbox/2/model.sdf", name: "wp08_toolbox1", pose: {position: {x: -3.907, y: 0.01, z: 0.0}}'
```

Eliminarlo:

```bash
gz service -s /world/NOMBRE_MUNDO/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "wp08_toolbox1", type: MODEL'
```

**Aviso sobre rutas con caracteres especiales:** algunos modelos de Fuel
tienen nombres de directorio con dos puntos (p. ej.
`drc practice: orange jersey barrier`). Encierra siempre `sdf_filename`
entre comillas dobles dentro del `--req`, o el shell puede interpretar
mal la ruta.

### 4. Escenarios actuales

| Escenario | Descripción |
|---|---|
| `escenario_A.yaml` | Estado base: todos los puntos con señal presente (boyas rojas en wp01). |
| `escenario_B.yaml` | Degradación parcial: boyas verdes en wp01; wp02, wp05, wp06 y wp08 vaciados parcial o totalmente; wp03/wp04 sin cambios. |
| `escenario_C.yaml` | Degradación en wp02/wp03/wp04; recuperación de wp05/wp06/wp08 al estado de A; wp01 se mantiene en verde. |

Entre los tres escenarios, cada waypoint mutable pasa por estado
presente/ausente sin repetir la misma combinación completa — pensado
como evidencia en la defensa de que la detección responde al estado real
de la escena y no a un resultado memorizado.

### 5. Pendiente (TODO)

- **`scripts/cargar_escenario.py`**: script que debe hacer *diff* entre
  el escenario actualmente cargado y el nuevo YAML — crear lo que
  aparece, **eliminar lo que ya no está** — no solo iterar y crear el
  YAML nuevo, o quedarán elementos "fantasma" de escenarios anteriores.
- **`eventos/barrera_wp07.yaml`**: guion final de disparo. Confirmado que
  ambas instancias de barrera forman una única zona cortada (un solo
  evento), pendiente decidir si se spawnean simultáneamente o con
  desfase temporal dentro del script de evento.