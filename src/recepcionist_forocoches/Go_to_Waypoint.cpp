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

#include "recepcionist_forocoches/Go_to_Waypoint.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;
using std::placeholders::_2;

Go_to_Waypoint::Go_to_Waypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);

  // Building Action Client
  action_client_ =
    rclcpp_action::create_client<recepcionist_forocoches::Go_to_Waypoint::NavigateToPose>(
    node_, "navigate_to_pose");

  // Building goal
  goal_ = NavigateToPose::Goal();
  goal_.pose.header.frame_id = "map";
}

void
Go_to_Waypoint::halt()
{
}

BT::NodeStatus
Go_to_Waypoint::tick()
{
  // --- Initializing Action (if inactive)---
  if (status() == BT::NodeStatus::IDLE) {
    // --- Settling pose ---
    geometry_msgs::msg::PoseStamped wp;
    getInput("waypoint", wp);
    goal_.pose = wp;

    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(
      node_->get_logger(), "Info: %lf,%lf", goal_.pose.pose.position.x,
      goal_.pose.pose.position.y);

    // --- Initializing Action ---
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&Go_to_Waypoint::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Go_to_Waypoint::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Go_to_Waypoint::result_callback, this, _1);

    // --- Starting Navigation ---
    RCLCPP_INFO(node_->get_logger(), "Sending goal...");
    action_client_->async_send_goal(goal_, send_goal_options);
    bt_status_ = BT::NodeStatus::RUNNING;
  }

  return bt_status_;
}

void
Go_to_Waypoint::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    bt_status_ = BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted. Navigating to waypoint");
    bt_status_ = BT::NodeStatus::RUNNING;
  }
}

void
Go_to_Waypoint::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "Feedback received: %d", feedback.get()->number_of_recoveries);
  (void)feedback;
}

void
Go_to_Waypoint::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal achieved!!");
      bt_status_ = BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      bt_status_ = BT::NodeStatus::FAILURE;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      bt_status_ = BT::NodeStatus::FAILURE;
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      bt_status_ = BT::NodeStatus::FAILURE;
      return;
  }
}
}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Go_to_Waypoint>("Go_to_Waypoint");
}
