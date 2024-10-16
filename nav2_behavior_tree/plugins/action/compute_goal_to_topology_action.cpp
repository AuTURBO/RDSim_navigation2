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

#include "nav2_behavior_tree/plugins/action/compute_goal_to_topology_action.hpp"

namespace nav2_behavior_tree {

ComputeGoalToTopologyAction::ComputeGoalToTopologyAction(const std::string &name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  topology_map_client_ = node_->create_client<nav2_msgs::srv::GetTopologyMap>("get_topology_map");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

BT::NodeStatus ComputeGoalToTopologyAction::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  auto request = std::make_shared<nav2_msgs::srv::GetTopologyMap::Request>();
  auto result = topology_map_client_->async_send_request(request);
  int start_id;
  int end_id;
  nav2_msgs::msg::TopologyMap topology_map;

  getInput("start_vertex_id", start_id);
  getInput("end_vertex_id", end_id);
  getInput("input_goals", input_goals);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) == rclcpp::FutureReturnCode::SUCCESS) {
    topology_map = result.get()->topology;
  }
  const int map_size = topology_map.vertices.size();

  if (start_id != topology_goal_start_ || end_id != topology_goal_end_) {
    cached_topology_path_.clear();
  }

  if (!input_goals.empty() && start_id == topology_goal_start_ && end_id == topology_goal_end_) {
    setOutput("output_goals", input_goals);
    return BT::NodeStatus::SUCCESS;
  }

  std::priority_queue<std::pair<int, int>> pq; // vertex_id, distance
  std::vector<std::pair<int, int>> table(map_size,
                                         {-1, std::numeric_limits<int>::max()}); // previous vertex id, min_distance
  table.resize(map_size);
  pq.push({start_id, 0});
  table[start_id] = {start_id, 0};

  while (!pq.empty()) {
    auto [cur_id, cur_dist] = pq.top();
    cur_dist *= -1;
    pq.pop();

    if (cur_dist > table[cur_id].second) {
      continue;
    }

    for (int i = 0; i < map_size; ++i) {
      int dist = topology_map.edges[cur_id * map_size + i];
      int next_id = i;
      int next_dist = cur_dist + dist;

      if (table[next_id].second > next_dist && dist != 0) {
        table[next_id] = {cur_id, next_dist};
        pq.push({next_id, -next_dist});
      }
    }
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = node_->get_clock()->now();
  int cur_vertex_id = end_id;

  double prev_x = topology_map.vertices[table[end_id].first].pose.position.x;
  double prev_y = topology_map.vertices[table[end_id].first].pose.position.y;
  double cur_x = topology_map.vertices[end_id].pose.position.x;
  double cur_y = topology_map.vertices[end_id].pose.position.y;

  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(cur_y - prev_y, cur_x - prev_x));

  for (;;) {
    pose_stamped.header.stamp = node_->get_clock()->now();
    pose_stamped.pose = topology_map.vertices[cur_vertex_id].pose;
    
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    goals.push_back(pose_stamped);
    if (table[cur_vertex_id].first == -1 || cur_vertex_id == start_id) {
      break;
    }

    cur_vertex_id = table[cur_vertex_id].first;
  }
  bool valid_path = cur_vertex_id == start_id;

  if (!valid_path) {
    goals.clear();
  }

  std::reverse(goals.begin(), goals.end());

  cached_topology_path_ = std::move(goals);
  topology_goal_start_ = start_id;
  topology_goal_end_ = end_id;

  setOutput("output_goals", cached_topology_path_);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<nav2_behavior_tree::ComputeGoalToTopologyAction>(name, config);
  };

  factory.registerBuilder<nav2_behavior_tree::ComputeGoalToTopologyAction>("ComputeGoalToTopology", builder);
}