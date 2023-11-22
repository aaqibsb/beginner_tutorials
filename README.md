# ROS2 Humble - Simple Publisher and Subscriber with Service

## Overview

This ROS2 workspace contains two packages:

1. first_publisher_subscriber: Publishes and subcribes to a topic with service-client functionality.
2. custom_msg_srv: Hosts custom message(.msg) and service(.srv) files

Results folder has the cpplint and cppcheck outputs along with screenshots of the commands running in the terminal.

## Dependencies

Please make sure the following prerequisites are met before running this package on your system:

- ROS2 Humble
- Ubuntu 22.04
- VSCode

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
    ros2 run first_publisher_subscriber talker
    ```

5. **Run the Subscriber Node**
    Open a new terminal and run the Subscriber node.

    ```sh
    source install/setup.bash
    ros2 run beginner_tutorials_ros2 subscriber_member_function.cpp
    ```
