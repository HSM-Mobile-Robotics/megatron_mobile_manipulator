from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():
    basic_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('megatron_bringup'), 'launch', 'basic_robot.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('megatron_bringup'), 'config', 'ekf.yaml']
    )

    imu_config_path = PathJoinSubstitution(
        [FindPackageShare('megatron_bringup'), 'config', 'imu_madgwick_filter.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='micro_ros_serial_port', 
            default_value='/dev/ttyACM0',
            description='Megatron teensy Serial Port'
        ),

        DeclareLaunchArgument(
            name='micro_ros_baudrate', 
            default_value='460800',
            description='micro-ROS baudrate'
        ),

        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value='serial',
            description='micro-ROS transport'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basic_robot_launch_path),
            launch_arguments={
                'micro_ros_serial_port': LaunchConfiguration("micro_ros_serial_port")
            }.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link',
            arguments=['0.254', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_config_path]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ]
        )
    ])