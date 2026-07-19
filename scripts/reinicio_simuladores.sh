pkill -9 -f "gz sim"
pkill -9 -f slam_toolbox
pkill -9 -f rviz2
pkill -9 -f teleop
pkill -9 -f parameter_bridge
pkill -9 -f robot_state_publisher

ps aux | grep -E "gz|slam|rviz|bridge|state_publisher" | grep -v grep

ros2 daemon stop && ros2 daemon start