// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file simple_test.cpp
 * @author Aaqib Barodawala
 * @brief Simple test case to showcase gtest framework functionality with ROS2
 * @version 0.1
 * @date 2023-28-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include <custom_msg_srv/msg/custom_msg.hpp>

/**
 * @brief Definition and implementation of TaskPlanningFixture class for testing ROS2 functionality with gtest.
 * 
 */
class TaskPlanningFixture : public testing::Test {
 public:
  /**
   * @brief Constructor for TaskPlanningFixture
   * 
   */
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("simple_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Constructor Completed!");
  }

  /**
   * @brief Set up actions that should occur before every test instance.
   * 
   */
  void SetUp() override {
    bool start_result = StartROSExec(
        "first_publisher_subscriber",
        "minimal_publisher",
        "talker");
    ASSERT_TRUE(start_result);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Setup Completed!");
  }

  /**
   * @brief Tear down actions that should occur after every test instance.
   * 
   */
  void TearDown() override {
    bool stop_result = StopROSExec();
    ASSERT_TRUE(stop_result);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Teardown Completed!");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;
 /**
   * @brief Start a ROS2 executable for testing.
   * @param pkg_name Name of the ROS2 package.
   * @param node_name Name of the ROS2 node.
   * @param exec_name Name of the executable.
   * @return True if the executable is started successfully, false otherwise.
   */
  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // Build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, sizeof(execName), "%s",
             exec_name);  // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // Kill the ROS2 node, incase it's still running.
    StopROSExec();

    // Start a ROS2 node and wait for it to get ready:
    int return_val = system(cmd_ss.str().c_str());
    if (return_val != 0) return false;

    return_val = -1;
    while (return_val != 0) {
      return_val = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }

    return true;
  }

  /**
   * @brief Stop the running ROS2 executable
   * 
   * @return true if stopped successfully
   * @return false otherwise
   */
  bool StopROSExec() {
    if (killCmd_ss.str().empty()) return true;

    int return_val = system(killCmd_ss.str().c_str());
    return return_val == 0;
  }
};

/**
 * @brief Test fixture for ROS2 functionality using gtest.
 */
TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "Starting TEST!" << std::endl;
  EXPECT_TRUE(true);

  // Subscribe to topic
  using SUBSCRIBER =
    rclcpp::Subscription<custom_msg_srv::msg::CustomMsg>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription =
      node_->create_subscription<custom_msg_srv::msg::CustomMsg>(
          "topic", 10,
          // Lambda expression begins
          [&](const custom_msg_srv::msg::CustomMsg& msg) {
            RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.txt.c_str());
            hasData = true;
          });


  // Check whether we get data within 6 seconds
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < 6s) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "ROS Shutdown Complete!" << std::endl;
  return result;
}
