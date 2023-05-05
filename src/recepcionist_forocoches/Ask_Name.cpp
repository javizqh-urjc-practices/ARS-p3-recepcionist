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
using std::placeholders::_2;

Ask_Name::Ask_Name(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Settling blackboard
  config().blackboard->get("node", node_);


}

void
Ask_Name::halt()
{
}

BT::NodeStatus
Ask_Name::tick()
{

}


}  // namespace recepcionist_forocoches
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_forocoches::Ask_Name>("Ask_Name");
}
