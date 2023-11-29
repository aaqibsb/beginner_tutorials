from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    # Create launch description
    return LaunchDescription([
        # Declare the publisher rate as launch argument
        DeclareLaunchArgument(
            "publisher_rate",
            default_value="200",
            description="Publisher rate"
        ),

        # Declare the logging level as launch argument
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("info")),
            description="Logging level"
        ),

        # Launch argument that enables or disables ROS 2 bag recording
        DeclareLaunchArgument(
            "is_record_bag",
            default_value='false',
            description='Determines if ROS bag record should be enabled.'
        ),

        # Launch argument that sets the ROS 2 bag recording path
        DeclareLaunchArgument(
            "bag_file_path",
            default_value='rosbag/talker',
            description='Determines the location to save the bag file.'
        ),

        # Launch ros2 bag recorder
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('bag_file_path')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('is_record_bag'))
        ),

        # Launch Service node
        Node(
            package="first_publisher_subscriber",
            executable="service",
            name="service",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Launch Talker node
        Node(
            package="first_publisher_subscriber",
            executable="talker",
            name="talker",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[{"pub_rate": LaunchConfiguration("publisher_rate")}]
        ),

        # Launch Listener node
        Node(
            package="first_publisher_subscriber",
            executable="listener",
            name="listener",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
