"""
warehouse_tb3.launch.py

Arranca el mundo 'warehouse_turtlebot3.sdf' (almacén Fuel sin Tugbot)
y spawnea un TurtleBot3 Waffle dentro, replicando lo que hace
turtlebot3_gazebo/launch/turtlebot3_world.launch.py pero con este mundo.

Uso:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch warehouse_tb3.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Rutas ---
    # Carpeta de este propio repo (~/TFG), calculada relativa a este archivo
    tfg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(tfg_dir, 'worlds', 'warehouse_turtlebot3.sdf')

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # TURTLEBOT3_MODEL debe venir del entorno (export TURTLEBOT3_MODEL=waffle),
    # igual que exige spawn_turtlebot3.launch.py original.
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    urdf_path = os.path.join(
        turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf'
    )
    bridge_params = os.path.join(
        turtlebot3_gazebo_dir, 'params', model_folder + '_bridge.yaml'
    )

    # --- 1. Servidor de Gazebo, con NUESTRO mundo en vez de turtlebot3_world.world ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --- 2. Cliente gráfico de Gazebo (ventana 3D) ---
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
    )

    # --- 3. Robot state publisher (TF del robot) ---
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # --- 4. Spawnear el TurtleBot3 ---
    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1',
        ],
        output='screen',
    )

    # --- 5. Bridge Gazebo <-> ROS2 (clock, cmd_vel, odom, scan, imu, tf...) ---
    # Reutiliza el MISMO yaml que trae turtlebot3_gazebo instalado, no uno propio.
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # --- 6. Bridge de la cámara (no aplica al Burger, que no lleva cámara) ---
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # --- 7. GZ_SIM_RESOURCE_PATH: que Gazebo sepa dónde están los modelos ---
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models'),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    if TURTLEBOT3_MODEL != 'burger':
        ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(set_env_vars_resources)

    return ld
