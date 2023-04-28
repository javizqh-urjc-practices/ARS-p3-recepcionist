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

#include "recepcionist_forocoches/Get_Waypoint.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT

Get_Waypoint::Get_Waypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  geometry_msgs::msg::PoseStamped wp[4];

  for (int i = 0; i < 4; i++) {
    wp[i].header.frame_id = "map";
    wp[i].pose.orientation.w = 1;
  }

  // TODO: Use params and enums for simplifying coords
  // door wp
  wp[0].pose.position.x = 3;
  wp[0].pose.position.y = 3;
  door_point_ = wp[0];

  // party wp
  wp[1].pose.position.x = 2;
  wp[1].pose.position.y = 2;
  party_point_ = wp[1];

  // bar wp
  wp[2].pose.position.x = 1;
  wp[2].pose.position.y = 1;
  bar_point_ = wp[2];

  // person wp
  wp[3].pose.position.x = 0.5;
  wp[3].pose.position.y = 0.5;
  person_point_ = wp[3];
}

void
Get_Waypoint::halt()
{
}

BT::NodeStatus
Get_Waypoint::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  std::string id;
  getInput("wp_id", id);

  if (id == "door") {
    RCLCPP_INFO(node_->get_logger(), "DOOR");
    setOutput("waypoint", door_point_);
  } else if (id == "party") {
    RCLCPP_INFO(node_->get_logger(), "PARTY");
    setOutput("waypoint", party_point_);
  } else if (id == "bar") {
    RCLCPP_INFO(node_->get_logger(), "BAR");
    setOutput("waypoint", bar_point_);
  } else if (id == "person") {
    RCLCPP_INFO(node_->get_logger(), "PERSON");
    setOutput("waypoint", party_point_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid waypoint requested");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Get_Waypoint>("Get_Waypoint");
}
