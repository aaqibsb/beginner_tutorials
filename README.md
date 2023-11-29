# ROS2 Humble - Simple Publisher and Subscriber with Service, tf2, rosbag and Unit Testing

## Overview

This ROS2 workspace contains two packages:

1. first_publisher_subscriber: Publishes and subcribes to a topic with service-client functionality.
2. custom_msg_srv: Hosts custom message(.msg) and service(.srv) files

Results folder has the cpplint and cppcheck outputs along with screenshots of all the commands running in the terminal.

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

5. **Run the Service Node**

    Run the Service Node.

    ```sh
    ros2 run first_publisher_subscriber service
    ```

6. **Run the Publisher Node**

    Open a new terminal and run the Publisher node.

    ```sh
    ros2 run first_publisher_subscriber talker
    ```

7. **Run the Subscriber Node**

    Open a new terminal and run the Subscriber node.

    ```sh
    source install/setup.bash
    ros2 run first_publisher_subscriber listener
    ```

8. **To Launch all the nodes at once**

    Close the previous working terminals and open a new terminal.
    Paste the following command in the terminal to make use of the launch file.

    ```sh
    source install/setup.bash
    ros2 launch first_publisher_subscriber srv_launch.py
    ```

9. **To make a service call**

    Close the previous working terminals and open a new terminal.
    Paste the following command in the terminal to make use of the service.

    ```sh
    source install/setup.bash
    ros2 run first_publisher_subscriber service
    ```

    Open a new terminal and paste the following.

    ```sh
    source install/setup.bash
    ros2 service call /create_output custom_msg_srv/srv/CustomSrv "{name: 'WallE', talk: 'WAAAAAAALLLEEEEE'}"
    ```

10. **To check the tf2 tree**

    Open a terminal and run the Publisher node.

    ```sh
    source install/setup.bash
    ros2 run first_publisher_subscriber talker
    ```

    Open a new terminal and run the following commands to verify tf2 tree.

    ```sh
    # to view frames in the terminal
    ros2 run tf2_ros tf2_echo world talk

    # to save frames in a pdf format
    ros2 run tf2_tools view_frames
    ```

11. **To run test cases**

    Open a terminal and run the following commands in a terminal.

    ```sh
    # to run the test cases
    colcon test --packages-select first_publisher_subscriber
    
    # to view the test results
    cat log/latest_test/first_publisher_subscriber/stdout_stderr.log
    ```

12. **To launch and enable rosbag recorder**

    Open a terminal and run the following commands in a terminal.

    ```sh
    source install/setup.bash
    ros2 launch first_publisher_subscriber srv_launch.py  is_record_bag:=true  bag_file_path:=rosbag/talker
    ```

13. **To inspect rosbag recorder**

    Open a terminal and run the following commands in a terminal. Here, The rosbag file was moved to the results folder.

    ```sh
    source install/setup.bash
    ros2 bag info ./src/results/rosbag/talker 
    ```

14. **To launch rosbag playback**

    Open a terminal and run the following commands in a terminal.

    ```sh
    source install/setup.bash
    ros2 launch first_publisher_subscriber rb_replay_launch.py bag_file_path:=./src/results/rosbag/talker 
    ```