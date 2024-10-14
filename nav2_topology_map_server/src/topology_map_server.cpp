#include "nav2_topology_map_server/topology_map_server.hpp"

namespace nav2_topology_map_server {
TopologyMapServer::TopologyMapServer(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("topology_map_server", "", options) {
  RCLCPP_INFO(get_logger(), "TopologyMapServer node has been started.");

  declare_parameter("topology_yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "topology_map/path");
  declare_parameter("service_name", "get_topology_map");
  declare_parameter("frame_id", "map");
}

nav2_util::CallbackReturn TopologyMapServer::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Configuring");
  /**
   * TODO: Create Topic/Service/Action
   */

  std::string yaml_filename{get_parameter("topology_yaml_filename").as_string()};
  std::string topic_name{get_parameter("topic_name").as_string()};
  std::string service_name{get_parameter("service_name").as_string()};
  frame_id_ = get_parameter("frame_id").as_string();

  RCLCPP_INFO(get_logger(), "topology_filename: %s", yaml_filename.c_str());

  load_map_yaml_file(yaml_filename);

  topology_service_ = create_service<nav2_msgs::srv::GetTopologyMap>(
      service_name,
      [&]([[maybe_unused]] const std::shared_ptr<nav2_msgs::srv::GetTopologyMap::Request> request,
          std::shared_ptr<nav2_msgs::srv::GetTopologyMap::Response> response) -> void {
        if (map_updated_) {
          cached_topology_map_.header.frame_id = frame_id_;
          cached_topology_map_.header.stamp = get_clock()->now();
          cached_topology_map_.vertices = vertices_;

          std::vector<int16_t> edges;
          for (size_t i = 0; i < edges_.size(); ++i) {
            for (size_t j = 0; j < edges_[i].size(); ++j) {
              edges.push_back(edges_[i][j]);
            }
          }

          cached_topology_map_.edges = edges;
          map_updated_ = false;
        }
        response->topology = cached_topology_map_;
      });
  map_visual_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopologyMapServer::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  /**
   * TODO: Activate Topic
   */
  map_visual_pub_->on_activate();
  if (map_available_) {
    auto visual_map{get_visual_map()};
    map_visual_pub_->publish(visual_map);
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopologyMapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  /**
   * TODO: Deactivate Topic
   */

  // destroy bond connection
  map_visual_pub_->on_deactivate();
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopologyMapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  map_visual_pub_.reset();
  topology_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopologyMapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void TopologyMapServer::load_map_yaml_file(const std::string &yaml_filename) {
  if (yaml_filename.empty()) {
    RCLCPP_INFO(get_logger(), "Topology map empty!!");
    return;
  }

  const auto config = YAML::LoadFile(yaml_filename);
  const auto vertex_size = config["topology_map"]["vertexs"].size();

  vertices_.reserve(vertex_size);

  for (const auto &vertexNode : config["topology_map"]["vertexs"]) {
    nav2_msgs::msg::TopologyVertex vertex;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Pose pose;

    pose.position.x = vertexNode["pose"]["x"].as<double>();
    pose.position.y = vertexNode["pose"]["y"].as<double>();
    pose.position.z = vertexNode["pose"]["z"].as<double>();
    pose.orientation.w = 1.0;

    vertex.id = vertexNode["id"].as<int>();
    vertex.pose = std::move(pose);
    vertex.status = vertexNode["status"].as<std::string>();

    vertices_.push_back(std::move(vertex));
  }

  // Parse edges
  const YAML::Node &edgesNode = config["topology_map"]["edges"];
  int size = static_cast<int>(std::sqrt(edgesNode.size()));
  edges_.resize(size, std::vector<int>(size));

  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      edges_[i][j] = edgesNode[i * size + j].as<int>();
    }
  }

  map_available_ = true;
}

visualization_msgs::msg::MarkerArray TopologyMapServer::get_visual_map() {
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker line_list_marker;
  visualization_msgs::msg::Marker vertex_marker;
  visualization_msgs::msg::Marker label_marker;

  line_list_marker.header.frame_id = "map";
  line_list_marker.header.stamp = this->get_clock()->now();
  line_list_marker.ns = "edges";
  line_list_marker.id = 0;
  line_list_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list_marker.action = visualization_msgs::msg::Marker::ADD;
  line_list_marker.scale.x = 0.4; // Line width

  vertex_marker.header.frame_id = "map";
  vertex_marker.header.stamp = this->get_clock()->now();
  vertex_marker.ns = "vertices";
  vertex_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  vertex_marker.action = visualization_msgs::msg::Marker::ADD;
  vertex_marker.scale.x = 0.4; // Sphere diameter
  vertex_marker.scale.y = 0.4;
  vertex_marker.scale.z = 0.1;
  vertex_marker.color.r = 0.0; // White color for text
  vertex_marker.color.g = 1.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0; // Full opacity

  label_marker.header.frame_id = "map";
  label_marker.header.stamp = this->get_clock()->now();
  label_marker.ns = "labels";
  label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label_marker.action = visualization_msgs::msg::Marker::ADD;
  label_marker.scale.z = 0.3; // Text size
  label_marker.color.r = 0.0; // White color for text
  label_marker.color.g = 0.0;
  label_marker.color.b = 0.0;
  label_marker.color.a = 0.5; // Full opacity

  double min_weight = std::numeric_limits<double>::max();
  double max_weight = std::numeric_limits<double>::min();

  for (const auto &row : edges_) {
    for (double weight : row) {
      if (weight > 0) { // Only consider positive weights
        min_weight = std::min(min_weight, weight);
        max_weight = std::max(max_weight, weight);
      }
    }
  }

  // Add vertex markers
  int vertex_id = 0;
  for (const auto &vertex : vertices_) {
    vertex_marker.id = vertex_id++; // Unique ID for each vertex marker
    vertex_marker.pose.position = vertex.pose.position;
    marker_array.markers.push_back(vertex_marker);

    label_marker.id = vertex_id; // Unique ID for each text marker
    label_marker.pose.position = vertex.pose.position;
    label_marker.pose.position.z = 0.2;
    label_marker.text = "ID: " + std::to_string(vertex.id);

    marker_array.markers.push_back(label_marker);
  }

  // Add lines between connected points based on the edges matrix
  for (size_t i = 0; i < edges_.size(); ++i) {
    for (size_t j = 0; j < edges_[i].size(); ++j) {
      if (edges_[i][j] >= 1) { // There is a connection between vertex i and j
        // double normalized_weight = (edges_[i][j] - min_weight) / (max_weight - min_weight);
        std_msgs::msg::ColorRGBA color;
        color.a = 0.2; // Full opacity
        color.r = 0.0; // Red decreases as weight increases
        color.g = 1.0; // Green increases as weight increases
        color.b = 0.0; // Blue remains 0 for the red-to-green gradient

        line_list_marker.points.push_back(vertices_[i].pose.position);
        line_list_marker.points.push_back(vertices_[j].pose.position);
        line_list_marker.colors.push_back(color); // Color for the start of the line
        line_list_marker.colors.push_back(color); // Color for the end of the line
      }
    }
  }

  marker_array.markers.push_back(line_list_marker);

  return marker_array;
}

TopologyMapServer::~TopologyMapServer() { RCLCPP_INFO(this->get_logger(), "TopologyMapServer node is shutting down."); }
} // namespace nav2_topology_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_topology_map_server::TopologyMapServer)