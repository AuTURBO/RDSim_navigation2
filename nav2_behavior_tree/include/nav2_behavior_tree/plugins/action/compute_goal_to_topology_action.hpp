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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_GOAL_TO_TOPOLOGY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_GOAL_TO_TOPOLOGY_ACTION_HPP_

#include <algorithm>
#include <queue>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/topology_map.hpp"
#include "nav2_msgs/srv/get_topology_map.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathToPose
 */
class ComputeGoalToTopologyAction : public BT::ActionNodeBase {
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  /**
   * @brief A constructor for nav2_behavior_tree::ComputeGoalToTopologyAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeGoalToTopologyAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);
  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
        BT::InputPort<Goals>("input_goals", "Goals with passed viapoints removed"),
        BT::InputPort<int>("start_vertex_id", "start_vertex_id"),
        BT::InputPort<int>("end_vertex_id", "end_vertex_id"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::GetTopologyMap>::SharedPtr topology_map_client_;
  std::chrono::milliseconds server_timeout_;

  std::vector<geometry_msgs::msg::PoseStamped> cached_topology_path_;
  int topology_goal_start_{-1};
  int topology_goal_end_{-1};
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_GOAL_TO_TOPOLOGY_ACTION_HPP_
