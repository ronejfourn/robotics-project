import os, xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare(package='robotics-project').find('robotics-project')

    gazebo_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_world = os.path.join(pkg_share, 'worlds', 'default.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=[('world', gazebo_world)]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_desc = xacro.process_file(robot_path).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }],
    )

    rviz_cfg = os.path.join(pkg_share, 'rviz/urdf.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ])
