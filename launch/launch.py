import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='robotics-project').find('robotics-project')
    default_model_path = os.path.join(pkg_share, 'urdf/robotics_project.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')

    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    decl_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
    )

    decl_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
    )

    decl_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
    )

    decl_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
    )

    decl_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
    )

    decl_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
    )

    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])
        }],
        arguments=[default_model_path]
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()

    ld.add_action(decl_model_path_cmd)
    ld.add_action(decl_rviz_config_file_cmd)
    ld.add_action(decl_use_joint_state_publisher_cmd)
    ld.add_action(decl_use_robot_state_pub_cmd)
    ld.add_action(decl_use_rviz_cmd)
    ld.add_action(decl_use_sim_time_cmd)

    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
