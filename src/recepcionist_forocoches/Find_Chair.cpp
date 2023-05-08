// Copyright 2021 Intelligent Robotics Lab
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

#include "recepcionist_forocoches/Find_Chair.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

Find_Chair::Find_Chair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // Building subscriptor
  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&Find_Chair::detection_callback, this, _1));

  // Building Publisher
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Building tf listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
}

void Find_Chair::detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  last_detection_ = std::move(msg);
}

void
Find_Chair::halt()
{
}


BT::NodeStatus
Find_Chair::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Searching Chair...");

  // Check if has detections
  if (last_detection_ == nullptr) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.4;
    vel_pub_->publish(twist);
    return BT::NodeStatus::RUNNING;
  }

  // For each detection
  for (auto detection : last_detection_->detections) {
    if (detection.results[0].hypothesis.class_id.compare("chair") == 0) {
      RCLCPP_INFO(node_->get_logger(), "Chair Detected");
      // Tf from robot to chair
      tf2::Transform robot2chair;
      robot2chair.setOrigin(
        tf2::Vector3(
          detection.bbox.center.position.z,
          -detection.bbox.center.position.x, 0.0));
      robot2chair.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      // Tf from map to robot
      geometry_msgs::msg::TransformStamped map2robot_msg;
      tf2::Stamped<tf2::Transform> map2robot;
      try {
        map2robot_msg = tf_buffer_->lookupTransform(
          "map", "base_link",
          tf2::timeFromSec(rclcpp::Time(last_detection_->header.stamp).seconds() - 0.3));
        tf2::fromMsg(map2robot_msg, map2robot);
      } catch (tf2::TransformException & ex) {
        return BT::NodeStatus::RUNNING;
        RCLCPP_INFO(node_->get_logger(), "LookupTransform from map to base_link failed");
      }

      // Final tf to send
      tf2::Transform map2chair = map2robot * robot2chair;
      geometry_msgs::msg::TransformStamped map2chair_msg;
      map2chair_msg.transform = tf2::toMsg(map2chair);

      // Printing and sending coordinates
      RCLCPP_INFO(
        node_->get_logger(), "Chair Coordinates: [%f,%f]", map2chair_msg.transform.translation.x,
        map2chair_msg.transform.translation.y);
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "map";
      wp.pose.position.x = map2chair_msg.transform.translation.x;
      wp.pose.position.y = map2chair_msg.transform.translation.y;
      wp.pose.orientation = map2chair_msg.transform.rotation;
      setOutput("chair", wp);

      return BT::NodeStatus::SUCCESS;
    }
  }

  geometry_msgs::msg::Twist twist;
  twist.angular.z = 0.4;
  vel_pub_->publish(twist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Find_Chair>("Find_Chair");
}
