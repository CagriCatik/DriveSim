import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    navpkg = 'autocar_nav'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'
    mappkg = 'autocar_map'

    # Paths
    gz_pkg_share = get_package_share_directory(gzpkg)
    world_path = os.path.join(gz_pkg_share, 'worlds', 'autocar.world')
    urdf_path = os.path.join(get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    rviz_config = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    nav_config = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Environment variables for Gazebo
    # We need to tell Gazebo where to find the models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(gz_pkg_share, 'models'), os.pathsep, os.path.dirname(gz_pkg_share)]
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # ROS-Gazebo Bridge
    # This bridges topics between ROS 2 and Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Ackermann Control
            '/model/autocar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Lidar Sensor
            '/world/default/model/autocar/link/hokuyo_link/sensor/head_hokuyo_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Odometry
            '/model/autocar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # TF
            '/model/autocar/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/autocar/cmd_vel', '/autocar/cmd_vel'),
            ('/world/default/model/autocar/link/hokuyo_link/sensor/head_hokuyo_sensor/scan', '/scan'),
            ('/model/autocar/odometry', '/autocar/odom'),
            ('/model/autocar/tf', '/tf')
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_path]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation Nodes
    localisation = Node(
        package=navpkg,
        executable='localisation.py',
        name='localisation',
        parameters=[nav_config, {'use_sim_time': use_sim_time}]
    )

    global_planner = Node(
        package=navpkg,
        executable='globalplanner.py',
        name='global_planner',
        parameters=[nav_config, {'use_sim_time': use_sim_time}]
    )

    local_planner = Node(
        package=navpkg,
        executable='localplanner.py',
        name='local_planner',
        parameters=[nav_config, {'use_sim_time': use_sim_time}]
    )

    bof = Node(
        package=mappkg,
        executable='bof',
        name='bof',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    path_tracker = Node(
        package=navpkg,
        executable='tracker.py',
        name='path_tracker',
        parameters=[nav_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        gz_resource_path,
        gazebo,
        bridge,
        robot_state_publisher,
        rviz,
        localisation,
        global_planner,
        local_planner,
        bof,
        path_tracker
    ])
