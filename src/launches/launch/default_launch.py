"""
default_launch.py
=================
Full autonomous simulation with the pre-recorded waypoint path.

Starts:
  - Gazebo Harmonic (gz sim) with autocar_road.world
  - Vehicle spawn at waypoint start position (103.67, 0, yaw=pi/2)
  - ROS-Gz bridge (cmd_vel, odom, tf, scan, clock)
  - robot_state_publisher
  - RViz2
  - localisation, global_planner, local_planner, path_tracker
  - bof (occupancy map)

Usage:
  ros2 launch launches default_launch.py
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    navpkg = 'autocar_nav'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'
    mappkg = 'autocar_map'

    gz_pkg_share = get_package_share_directory(gzpkg)
    model_sdf = os.path.join(gz_pkg_share, 'models', 'autocar', 'model.sdf')
    urdf_path = os.path.join(get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    rviz_config = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    nav_config = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='autocar_road.world')

    # Process URDF at launch-file load time (xacro → XML string)
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # VS Code runs as a snap and injects GTK_PATH / GTK_EXE_PREFIX pointing at
    # snap/core20 GLIBC libs. Those libs are incompatible with Ubuntu 24.04 and
    # cause RViz2 and Gazebo GUI to crash with "undefined symbol: __libc_pthread_init".
    # Unsetting them before any GUI process starts keeps the system GTK in use.
    unset_snap_gtk = [
        UnsetEnvironmentVariable('GTK_PATH'),
        UnsetEnvironmentVariable('GTK_EXE_PREFIX'),
        UnsetEnvironmentVariable('GDK_PIXBUF_MODULE_FILE'),
        UnsetEnvironmentVariable('GDK_PIXBUF_MODULEDIR'),
        UnsetEnvironmentVariable('GSETTINGS_SCHEMA_DIR'),
    ]

    # Make Gazebo Harmonic aware of the package's model directory
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(gz_pkg_share, 'models'),
            os.pathsep,
            os.path.dirname(gz_pkg_share),
        ]
    )

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', PathJoinSubstitution([gz_pkg_share, 'worlds', world_name])]
        }.items(),
    )

    # Spawn vehicle at the start of the waypoint path (103.67, 0) facing +Y
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'autocar',
            '-file', model_sdf,
            '-x', '103.67',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '1.5708',
        ],
        output='screen',
    )

    # ROS ↔ Gazebo bridge
    # Remapping strategy:
    #   /autocar/cmd_vel  (ROS, tracker) -> /model/autocar/cmd_vel (gz plugin input)
    #   /model/autocar/odometry (gz)     -> /autocar/odom (ROS, localisation + bof)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/autocar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/autocar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/autocar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # JointStatePublisher plugin publishes to /world/{world}/model/{model}/joint_state
            '/world/default/model/autocar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args',
            '-r', '/model/autocar/cmd_vel:=/autocar/cmd_vel',
            '-r', '/model/autocar/odometry:=/autocar/odom',
            '-r', '/model/autocar/scan:=/scan',
            '-r', '/world/default/model/autocar/joint_state:=/joint_states',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    localisation = Node(
        package=navpkg,
        executable='localisation.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    # default_launch uses global_planner (pre-recorded waypoints), NOT click_planner
    global_planner = Node(
        package=navpkg,
        executable='globalplanner.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    local_planner = Node(
        package=navpkg,
        executable='localplanner.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    path_tracker = Node(
        package=navpkg,
        executable='tracker.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    bof = Node(
        package=mappkg,
        executable='bof',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='autocar_road.world'),
        *unset_snap_gtk,
        gz_resource_path,
        gazebo,
        spawn_robot,
        bridge,
        robot_state_publisher,
        rviz,
        localisation,
        global_planner,
        local_planner,
        path_tracker,
        bof,
    ])
