from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Create launch description
    return LaunchDescription([

        # Declare the publisher rate as launch argument
        DeclareLaunchArgument(
            "publisher_rate",
            default_value = "500",
            description="Publisher rate"
        ),

        # Declare the logging level as launch argument
        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("info")),
            description="Logging level"
        ),

        # Launch Service node
        Node(
            package="first_publisher_subscriber",
            executable="service",
            name="service",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Laucnh Talker node
        Node(
            package="first_publisher_subscriber",
            executable="talker",
            name="talker",
            arguments=['--ros-args','--log-level', LaunchConfiguration('log_level')],
            parameters=[
            {"pub_rate": LaunchConfiguration("publisher_rate")}]
        ),

        # Laucnh Listener node
        Node(
            package="first_publisher_subscriber",
            executable="listener",
            name="listener",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )

    ])