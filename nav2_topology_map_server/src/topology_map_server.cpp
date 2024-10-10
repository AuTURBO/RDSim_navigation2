#include "nav2_topology_map_server/topology_map_server.hpp"

namespace nav2_topology_map_server {
TopologyMapServer::TopologyMapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("topology_map_server", "", options)
{
    RCLCPP_INFO(this->get_logger(), "TopologyMapServer node has been started.");
}
nav2_util::CallbackReturn
TopologyMapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  /**
   * TODO: Create Topic/Service/Action
   */

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TopologyMapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  /**
   * TODO: Activate Topic
   */

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TopologyMapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  /**
   * TODO: Deactivate Topic
   */

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TopologyMapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TopologyMapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

TopologyMapServer::~TopologyMapServer()
{
    RCLCPP_INFO(this->get_logger(), "TopologyMapServer node is shutting down.");
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_topology_map_server::TopologyMapServer)