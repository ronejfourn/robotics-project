import os, xacro, yaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = FindPackageShare(package='robotics-project').find('robotics-project')

    gazebo_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_world = os.path.join(pkg_share, 'worlds', 'obstacles.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=[('world', gazebo_world)]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robotics-project'],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description = xacro.process_file(robot_path).toxml()
    robot_description = { 'robot_description': robot_description }
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
    )

    gripper_l_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_l_controller', '-c', '/controller_manager'],
    )

    gripper_r_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_r_controller', '-c', '/controller_manager'],
    )

    diffdrive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller', '-c', '/controller_manager'],
    )

    robot_description_semantic = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(robot_description_semantic, 'r') as f:
        robot_description_semantic = { 'robot_description_semantic': f.read() }

    kinematics_yaml = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = { 'robot_description_kinematics': kinematics_yaml }

    ompl_planning_yaml = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_yaml, 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)
    ompl_planning = { 'move_group': ompl_planning_yaml }

    moveit_controllers = os.path.join(pkg_share, 'config', 'controller_moveit.yaml')
    with open(moveit_controllers, 'r') as f:
        moveit_controllers = {
            'moveit_simple_controller_manager': yaml.safe_load(f),
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    rviz_cfg = os.path.join(pkg_share, 'config', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    slam_toolbox_share = FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        slam_toolbox,
        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [ joint_state_broadcaster ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster,
                on_exit = [ arm_controller ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = arm_controller,
                on_exit = [ gripper_l_controller ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = gripper_l_controller,
                on_exit = [ gripper_r_controller ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = gripper_l_controller,
                on_exit = [ diffdrive_controller ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = gripper_r_controller,
                on_exit = [ rviz, run_move_group_node ]
            )
        ),
    ])
