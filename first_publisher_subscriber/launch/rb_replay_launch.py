from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Create launch description
    return LaunchDescription([
        # Launch argument that sets the ROS 2 bag recording path
        DeclareLaunchArgument(
            "bag_file_path",
            default_value='rosbag/talker',
            description='Determines the location to save the bag file.'
        ),

        # Launch ROS 2 bag player
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file_path')],
            output='screen'
        ),

        # Launch listener node
        Node(
            package="first_publisher_subscriber",
            executable="listener",
            name="listener"
        )
    ])
