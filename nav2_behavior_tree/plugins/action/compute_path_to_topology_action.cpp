// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_path_to_topology_action.hpp"

namespace nav2_behavior_tree {

ComputePathToTopologyAction::ComputePathToTopologyAction(const std::string &xml_tag_name,
                                                         const std::string &action_name,
                                                         const BT::NodeConfiguration &conf)
    : BtActionNode<nav2_msgs::action::ComputePathToTopology>(xml_tag_name, action_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  topology_map_client_ = node_->create_client<nav2_msgs::srv::GetTopologyMap>("get_topology_map");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

void ComputePathToTopologyAction::on_tick() {
  auto request = std::make_shared<nav2_msgs::srv::GetTopologyMap::Request>();
  auto result = topology_map_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) == rclcpp::FutureReturnCode::SUCCESS) {
    goal_.topology_map = result.get()->topology;
  }

  getInput("start_vertex_id", goal_.start_vertex_id);
  getInput("end_vertex_id", goal_.end_vertex_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathToTopologyAction::on_success() {
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToTopologyAction::on_aborted() {
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToTopologyAction::on_cancelled() {
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputePathToTopologyAction::halt() {
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<nav2_behavior_tree::ComputePathToTopologyAction>(name, "compute_path_to_topology", config);
  };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToTopologyAction>("ComputePathToTopology", builder);
}
