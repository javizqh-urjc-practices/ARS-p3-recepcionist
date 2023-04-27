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

  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // door wp
  wp.pose.position.x = 3.67;
  wp.pose.position.y = -0.24;
  door_point_ = wp;

  // party wp
  wp.pose.position.x = 1.07;
  wp.pose.position.y = -12.38;
  party_point_ = wp;

  // bar wp
  wp.pose.position.x = -5.32;
  wp.pose.position.y = -8.85;
  bar_point_ = wp;
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
    setOutput("waypoint", door_point_);
  } else if (id == "party") {
    setOutput("waypoint", party_point_);
  } else if (id == "bar") {
    setOutput("waypoint", bar_point_);
  } else if (id == "person") {
    setOutput("waypoint", party_point_);
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Get_Waypoint>("Get_Waypoint");
}
