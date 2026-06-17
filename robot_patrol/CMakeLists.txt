cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces inside this package
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

# Typesupport handle for linking generated interfaces into same-package C++ targets
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ── Existing Checkpoint 1 patrol node ──────────────────────────────────────
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp sensor_msgs geometry_msgs)

# ── Milestone 2: direction service server ──────────────────────────────────
add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs)
target_link_libraries(direction_service_node "${cpp_typesupport_target}")

# ── Milestone 2: test service client ───────────────────────────────────────
add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs)
target_link_libraries(test_service_node "${cpp_typesupport_target}")

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(patrol_with_service_node "${cpp_typesupport_target}")

install(TARGETS
  patrol_node
  direction_service_node
  test_service_node
  patrol_with_service_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()