# ROS2 Humble - Simple Publisher and Subscriber

## Overview

This ROS2 package publishes and subcribes to a topic with a custom string message.

## Dependencies

Please make sure the following prerequisites are met before running this package on your system:

- ROS2 Humble
- Ubuntu 22.04

## Build and Run Instructions

1. **Clone Package**

   Clone the repository/package by using the following command in the terminal. Make sure to place it in the `src` directory of your workspace.

   ```sh
   cd <your_ros2_workspace>/src
   git clone https://github.com/aaqibsb/beginner_tutorials.git
   ```

2. **Build the Package**

    Change the directory to your ROS2 workspace and build the package using colcon.

    ```sh
    cd ..
    colcon build
    ```

3. **Source the Workspace**

    Source the ROS2 workspace to set the environment for the package.

    ```sh
    source install/setup.bash
    ```

4. **Run the Publisher Node**

    Run the Publisher node.

    ```sh
    ros2 run first_publisher_subscriber publisher_member_function.cpp
    ```

5. **Run the Subscriber Node**

    Open a new terminal and run the Subscriber node.

    ```sh
    source install/setup.bash
    ros2 run first_publisher_subscriber subscriber_member_function.cpp
    ```
