cmake_minimum_required(VERSION 3.8)
project(nav2_topology_map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(nav2_util REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

nav2_package()

include_directories(include)

set(topology_map_server_executable topology_map_server)

add_executable(${topology_map_server_executable}
  src/main.cpp)

set(library_name ${map_server_executable}_core)

add_library(${library_name} SHARED
  src/topology_map_server.cpp)

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  yaml_cpp_vendor
  nav2_util)

ament_target_dependencies(${topology_map_server_executable}
  ${map_server_dependencies})

ament_target_dependencies(${library_name}
  ${map_server_dependencies})

target_link_libraries(${topology_map_server_executable}
  ${library_name})

rclcpp_components_register_nodes(${library_name} "nav2_topology_map_server::TopologyMapServer")

install(TARGETS
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${topology_map_server_executable}
RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
)
ament_export_dependencies(${map_server_dependencies})
ament_package()

