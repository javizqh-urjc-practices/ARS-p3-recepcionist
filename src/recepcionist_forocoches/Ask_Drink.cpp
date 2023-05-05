// Copyright 2023 Intelligent Robotics Lab
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

#include "recepcionist_forocoches/Ask_Drink.hpp"

namespace recepcionist_forocoches
{

using namespace std::chrono_literals;  // NOLINT
using std::placeholders::_1;
using std::placeholders::_2;

Ask_Drink::Ask_Drink(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);

  dialog_.registerCallback(std::bind(&Ask_Drink::askDrinkIntentCB,this, _1));

}

void Ask_Drink::askDrink(){
  sc_.say("What is your favourite drink?");
}

void Ask_Drink::askDrinkIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] AskForDrink: intent [%s]", result.intent.c_str()); //WelcomeIntent

  //Consigue el nombre
  auto name = result.parameters[0].value[0];
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] AskForDrink: drink = %s", name.c_str());

  dialog_.speak(result.fulfillment_text);
}

void
Ask_Drink::halt()
{
}

BT::NodeStatus
Ask_Drink::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    //dialog_.askDrink();
    dialog_.listen();
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());
  return BT::NodeStatus::SUCCESS;
}


}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Ask_Drink>("Ask_Drink");
}
