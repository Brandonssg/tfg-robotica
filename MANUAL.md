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
export TURTLEBOT3_MODEL=waffle
ros2 launch ~/TFG/launch/warehouse_tb3.launch.py
```

**Nota conocida**: el TurtleBot3 se renderiza en gris plano en este mundo (incompatibilidad
conocida entre las mallas `.dae` del paquete `turtlebot3_gazebo` y el motor de render Ogre2 de
Gazebo Harmonic). No afecta a sensores, TF ni navegación — verificado mediante los topics del
bridge (`/odom`, `/scan`, `/imu`, `/tf`, `/camera/camera_info`, `/cmd_vel`).


## SLAM: construcción del mapa

Con la simulación base lanzada (`warehouse_tb3.launch.py`), en otra terminal:

```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Mueve el robot con teleop para cubrir todo el mapa y, cuando esté completo, guárdalo:

```
ros2 run nav2_map_server map_saver_cli -f ~/TFG/maps/mapa_tfg
```

Esto genera `mapa_tfg.pgm` (imagen del mapa) y `mapa_tfg.yaml` (metadatos: resolución, origen).


