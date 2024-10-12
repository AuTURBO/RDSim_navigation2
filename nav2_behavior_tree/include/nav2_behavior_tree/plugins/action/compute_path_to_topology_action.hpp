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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_TOPOLOGY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_TOPOLOGY_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_topology.hpp"
#include "nav2_msgs/msg/topology_map.hpp"
#include "nav2_msgs/srv/get_topology_map.hpp"
#include "nav_msgs/msg/path.h"

namespace nav2_behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathToPose
 */
class ComputePathToTopologyAction : public BtActionNode<nav2_msgs::action::ComputePathToTopology> {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ComputePathToTopologyAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputePathToTopologyAction(const std::string &xml_tag_name, const std::string &action_name,
                              const BT::NodeConfiguration &conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::InputPort<int>("start_vertex_id", "start_vertex_id"),
        BT::InputPort<int>("end_vertex_id", "end_vertex_id"),
        BT::InputPort<nav2_msgs::msg::TopologyMap>("topology_map", "topology_map"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start",
                                                       "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
    });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::GetTopologyMap>::SharedPtr topology_map_client_;
  std::chrono::milliseconds server_timeout_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_TOPOLOGY_ACTION_HPP_
