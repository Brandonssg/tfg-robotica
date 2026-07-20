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
