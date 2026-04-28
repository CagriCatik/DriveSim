"""
click_launch.py
===============
Interactive simulation — place goals in RViz with '2D Goal Pose' and the
vehicle drives to them using the click_planner (cubic-spline path through
RViz-placed waypoints).

Starts:
  - Gazebo Harmonic (gz sim) with autocar_road.world
  - Vehicle spawn at waypoint start position (103.67, 0, yaw=pi/2)
  - ROS-Gz bridge (cmd_vel, odom, tf, scan, imu, front camera, clock)
  - robot_state_publisher
  - RViz2
  - EKF localization, localisation, click_planner, frenet_planner, path_tracker
  - camera preprocessor, lidar obstacle detector, safety gate
  - bof (occupancy map)

Usage:
  ros2 launch launches click_launch.py
Then in RViz use '2D Goal Pose' to set at least 2 waypoints.
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
    locpkg = 'autocar_localization'
    planpkg = 'autocar_planning'
    perceptionpkg = 'autocar_perception'
    safetypkg = 'autocar_safety'

    gz_pkg_share = get_package_share_directory(gzpkg)
    model_sdf = os.path.join(gz_pkg_share, 'models', 'autocar', 'model.sdf')
    urdf_path = os.path.join(get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    rviz_config = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    nav_config = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')
    ekf_config = os.path.join(get_package_share_directory(locpkg), 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='autocar_road.world')

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

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(gz_pkg_share, 'models'),
            os.pathsep,
            os.path.dirname(gz_pkg_share),
        ]
    )

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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/autocar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/autocar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/autocar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/autocar/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/autocar/camera/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # JointStatePublisher plugin publishes to /world/{world}/model/{model}/joint_state
            '/world/default/model/autocar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args',
            '-r', '/model/autocar/cmd_vel:=/autocar/cmd_vel',
            '-r', '/model/autocar/odometry:=/autocar/odom',
            '-r', '/model/autocar/scan:=/autocar/scan',
            '-r', '/model/autocar/imu:=/autocar/imu',
            '-r', '/model/autocar/camera/front/image_raw:=/autocar/camera/front/image_raw',
            '-r', '/world/default/model/autocar/joint_state:=/joint_states',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
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

    # click_launch uses click_planner (interactive RViz goal pose), NOT global_planner
    click_planner = Node(
        package=navpkg,
        executable='clickplanner.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    frenet_planner = Node(
        package=planpkg,
        executable='frenet_planner.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    path_tracker = Node(
        package=navpkg,
        executable='tracker.py',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    lidar_obstacle_detector = Node(
        package=perceptionpkg,
        executable='lidar_obstacle_detector.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    camera_preprocessor = Node(
        package=perceptionpkg,
        executable='camera_preprocessor.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    safety_gate = Node(
        package=safetypkg,
        executable='safety_gate.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
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
        ekf,
        localisation,
        click_planner,
        camera_preprocessor,
        lidar_obstacle_detector,
        frenet_planner,
        path_tracker,
        safety_gate,
        bof,
    ])
