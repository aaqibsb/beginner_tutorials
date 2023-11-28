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
 * @file subscriber_member_function.cpp
 * @author Aaqib Barodawala (aaqib.barodawala@gmail.com)
 * @brief Simple Subscriber for ROS2
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <custom_msg_srv/msg/custom_msg.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

/**
 * @brief ROS2 Subscriber
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing Subscriber.");

    /**
     * @brief Create Subscriber to "topic"
     *
     */
    subscription_ = this->create_subscription<custom_msg_srv::msg::CustomMsg>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback for "topic"
   *
   * @param msg
   */
  void topic_callback(const custom_msg_srv::msg::CustomMsg& msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.txt);
  }

  /**
   * @brief Pointer for Subscriber
   *
   */
  rclcpp::Subscription<custom_msg_srv::msg::CustomMsg>::SharedPtr subscription_;
};

/**
 * @brief main function for subscriber node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
