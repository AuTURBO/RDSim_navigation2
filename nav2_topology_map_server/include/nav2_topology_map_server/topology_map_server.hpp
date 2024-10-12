#ifndef NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_
#define NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_

#include "nav2_msgs/msg/topology_vertex.hpp"
#include "nav2_msgs/srv/get_topology_map.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace nav2_topology_map_server {

// Define the Vertex structure
struct Vertex {
  int id;
  geometry_msgs::msg::Point point;
  std::string status;
};
class TopologyMapServer : public nav2_util::LifecycleNode {
public:
  explicit TopologyMapServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~TopologyMapServer();

protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters
   * from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  void load_map_yaml_file(const std::string &yaml_filename);
  visualization_msgs::msg::MarkerArray get_visual_map();

  bool map_available_{false};
  bool map_updated_{true};

  nav2_msgs::msg::TopologyMap cached_topology_map_;
  std::string frame_id_;
  std::vector<nav2_msgs::msg::TopologyVertex> vertices_;
  std::vector<std::vector<int>> edges_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_visual_pub_;
  rclcpp::Service<nav2_msgs::srv::GetTopologyMap>::SharedPtr topology_service_;
};
} // namespace nav2_topology_map_server
#endif // NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_
