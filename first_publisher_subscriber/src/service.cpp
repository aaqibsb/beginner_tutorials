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
 * @file service.cpp
 * @author Aaqib Barodawala (aaqib.barodawala@gmail.com)
 * @brief Simple Service Server for ROS2
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <custom_msg_srv/srv/custom_srv.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Callback for Service
 *
 * @param request Two Input Strings
 *  1. name : Name of the Person
 *  2. talk : Speech of the Person
 * @param response One Output String
 *  1. output : <name> + " said : "+<talk>
 */
void create_output(
    const std::shared_ptr<custom_msg_srv::srv::CustomSrv::Request> request,
    std::shared_ptr<custom_msg_srv::srv::CustomSrv::Response> response) {
  /**
   * @brief Construct the Output Speech
   *
   */
  response->output = request->name + " said : " + request->talk;

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"),
      "Incoming Request\n: " << request->name << " speech: " << request->talk);
  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"),
      "Sending back Output Speech: [" << response->output << "]");
}

/**
 * @brief Main function for Server node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  /**
   * @brief Initialize Service node
   *
   */
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("create_output_server");

  /**
   * @brief Initialize Service inside the node
   *
   */
  rclcpp::Service<custom_msg_srv::srv::CustomSrv>::SharedPtr
      create_output_service =
          node->create_service<custom_msg_srv::srv::CustomSrv>("create_output",
                                                               &create_output);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "Ready to generate Output Speech.");

  /**
   * @brief Spin node
   *
   */
  rclcpp::spin(node);
  rclcpp::shutdown();
}
