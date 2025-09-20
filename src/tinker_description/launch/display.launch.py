import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
  urdf_pkg = 'tinker_urdf'
  urdf_share = get_package_share_directory(urdf_pkg)
  urdf_path = os.path.join(urdf_share, 'urdf', 'tinker_urdf.urdf')

  with open(urdf_path, 'r') as f:
    robot_description = f.read()

  rviz_config = os.path.join(urdf_share, 'urdf.rviz')

  joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen'
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description}]
  )

  rviz2 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config]
  )

  return LaunchDescription([
    joint_state_publisher_gui,
    robot_state_publisher,
    rviz2,
  ])


