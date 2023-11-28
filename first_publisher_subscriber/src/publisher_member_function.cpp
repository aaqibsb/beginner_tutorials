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
 * @file publisher_member_function.cpp
 * @author Aaqib Barodawala (aaqib.barodawala@gmail.com)
 * @brief Simple Publisher with tf2 and Service Client functionality for ROS2
 * @version 0.2
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <custom_msg_srv/msg/custom_msg.hpp>
#include <custom_msg_srv/srv/custom_srv.hpp>

using namespace std::chrono_literals;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief ROS2 Publisher with Service Client
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing Publisher...");

    /**
     * @brief Create Publisher to "topic"
     *
     */
    publisher_ =
        this->create_publisher<custom_msg_srv::msg::CustomMsg>("topic", 10);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating Service Client...");

    /**
     * @brief Create Service Client for "create_output" service
     *
     */
    service_client_ =
        this->create_client<custom_msg_srv::srv::CustomSrv>("create_output");

    /**
     * @brief Declare publisher rate as a parameter
     *
     */
    this->declare_parameter("publisher_rate", 200);

    /**
     * @brief Create a tf broadcaster instance
     * 
     */
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    /**
     * @brief Variable to store the publisher rate
     *
     */
    int publisher_rate = this->get_parameter("publisher_rate").as_int();

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating Timer Instance");

    /**
     * @brief Timer to publish at intervals
     *
     */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publisher_rate),
        std::bind(&MinimalPublisher::timer_callback, this));

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Calling the service..");

    /**
     * @brief Variable to store the message
     *
     */
    output_msg = get_output("Darth Vader", "Luke, I am your father!");

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Service Returned : " << output_msg);

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Finished Initialization of Publisher.");
  }

 private:
  /**
   * @brief Get the output
   *
   * @param _name Name
   * @param _talk Speech of the person
   * @return std::string Return complete sentence
   */
  std::string get_output(const std::string& _name, const std::string& _talk) {
    /**
     * @brief Output Speech value
     *
     */
    std::string output_speech = "";

    /**
     * @brief Wait for availability of service
     *
     */
    if (!this->service_client_->wait_for_service(1s)) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Service is not available, skipping publish...");
    } else {
      /**
       * @brief Create Service Request
       *
       */
      auto request =
          std::make_shared<custom_msg_srv::srv::CustomSrv::Request>();
      request->name = _name;
      request->talk = _talk;

      /**
       * @brief Call the service
       *
       */
      auto result = this->service_client_->async_send_request(request);

      /**
       * @brief Wait for the result
       *
       */
      if (spin_until_future_complete(this->get_node_base_interface(), result) ==
          FutureReturnCode::SUCCESS) {
        output_speech = result.get()->output;

        if (output_speech == "") {
          RCLCPP_WARN_STREAM(this->get_logger(),
                             "Service returned empty result!");
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Output Speech: " << output_speech);
      } else {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                            "Failed to call service");
      }
    }

    return output_speech;
  }


  /**
   * @brief Method to publish tranform from /world frame to /talk frame
   * 
   */
  void publish_transform() {
    /**
     * @brief Build geometry message
     * 
     */
    geometry_msgs::msg::TransformStamped t;

    /**
     * @brief Set time stamp and header frames
     *  parent : /world
     *  child  : /talk
     */
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    /**
     * @brief Set translation to (20, 50, 0)
     * 
     */
    t.transform.translation.x = 20.0;
    t.transform.translation.y = 50.0;
    t.transform.translation.z = 0.0;

    /**
     * @brief Set yaw rotation as -180 deg / -1.57 rad
     * 
     */
    tf2::Quaternion q;
    q.setRPY(0, 0, -1.57);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    /**
     * @brief Send transform
     * 
     */
    tf_broadcaster_->sendTransform(t);
  }


  /**
    * @brief Build the message. If service is not available, publish default message
    * 
    */
  void publish_message() {
    auto message = custom_msg_srv::msg::CustomMsg();

    if (!this->service_client_->wait_for_service(100ms)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Service is not available, Publishing Default Message!");
      message.txt = "Luke, I am your father!" + std::to_string(count_++);
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.txt);
      publisher_->publish(message);
    } else {
      message.txt = this->output_msg + " " + std::to_string(count_++);
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.txt);
      publisher_->publish(message);
    }
  }


  /**
   * @brief Timer callback that publishes messages and transform periodically
   * 
   */
  void timer_callback() {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Timer Callback called!");
      publish_message();
      publish_transform();
  }


  /**
   * @brief Timer parameter
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Pointer for the Broadcaster
   * 
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @brief Pointer for the Publisher
   *
   */
  rclcpp::Publisher<custom_msg_srv::msg::CustomMsg>::SharedPtr publisher_;

  /**
   * @brief Pointer for the Service Client
   *
   */
  rclcpp::Client<custom_msg_srv::srv::CustomSrv>::SharedPtr service_client_;

  /**
   * @brief Count to number the message
   *
   */
  size_t count_;

  /**
   * @brief Stores the output obtained from the service
   *
   */
  std::string output_msg = "";
};

/**
 * @brief main function for publisher node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
