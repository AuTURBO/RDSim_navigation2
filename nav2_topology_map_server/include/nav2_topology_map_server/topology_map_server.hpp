#ifndef NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_
#define NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_topology_map_server {
class TopologyMapServer: public nav2_util::LifecycleNode
{
public:
    explicit TopologyMapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~TopologyMapServer();

protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
};
}
#endif  // NAV2_TOPOLOLGY_MAP_SERVER__TOPOLOGY_MAP_SERVER_HPP_
