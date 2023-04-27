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

#ifndef RECEPCIONIST_FOROCOCHES__GO_TO_WAYPOINT_HPP_
#define RECEPCIONIST_FOROCOCHES__GO_TO_WAYPOINT_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_forocoches
{

class Go_to_Waypoint : public BT::ActionNodeBase
{
public:
  explicit Go_to_Waypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("waypoint")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
};

}  // namespace recepcionist_forocoches

#endif  // RECEPCIONIST_FOROCOCHES__GO_TO_WAYPOINT_HPP_
