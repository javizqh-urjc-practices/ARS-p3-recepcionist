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

#include "recepcionist_forocoches/Ask_Name.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;

Ask_Name::Ask_Name(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);
  dialog_.registerCallback(std::bind(&Ask_Name::askNameIntentCB, this, _1), "RequestName");
}

void Ask_Name::askNameIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] Ask_Name: intent [%s]", result.intent.c_str());

  auto name = result.parameters[0].value[0];
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] Ask_Name: name = %s", name.c_str());
  responded_ = true;
  name_ = name.c_str();

  dialog_.speak(result.fulfillment_text);
}

void
Ask_Name::halt()
{
}

BT::NodeStatus
Ask_Name::tick()
{
  responded_ = false;
  if (status() == BT::NodeStatus::IDLE) {
    dialog_.speak("What is your name?");
    dialog_.listen();
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());
  if (responded_) {
    setOutput("person_name", name_);
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}


}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Ask_Name>("Ask_Name");
}
