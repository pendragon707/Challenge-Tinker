rm -r build/ install/
colcon build --symlink-install
source install/setup.bash
ros2 launch motor_control Motor_control 
