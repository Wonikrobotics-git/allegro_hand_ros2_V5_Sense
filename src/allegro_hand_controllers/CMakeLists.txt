cmake_minimum_required(VERSION 3.5)
project(allegro_hand_controllers)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────────────────────────────────
# Dependencies
# ─────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(allegro_hand_driver REQUIRED)
find_package(bhand REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${allegro_hand_driver_INCLUDE_DIRS}
  ${bhand_INCLUDE_DIRS}
)

# ─────────────────────────────────────
# Legacy targets (original executable)
# ─────────────────────────────────────
add_library(allegro_node STATIC src/allegro_node.cpp)
ament_target_dependencies(
  allegro_node rclcpp sensor_msgs allegro_hand_driver
)

add_executable(allegro_node_grasp src/allegro_node_grasp.cpp)
target_link_libraries(allegro_node_grasp
  allegro_node
  yaml-cpp
  ${bhand_LIBRARIES}
)
ament_target_dependencies(allegro_node_grasp rclcpp sensor_msgs allegro_hand_driver yaml-cpp bhand)

# ─────────────────────────────────────
# Install
# ─────────────────────────────────────

install(TARGETS
  allegro_node
  allegro_node_grasp
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.sh"
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING PATTERN "*.stl"
)

install(DIRECTORY pose/
  DESTINATION share/${PROJECT_NAME}/pose
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
  PATTERN "*.rviz"
)

# ─────────────────────────────────────
# Export
# ─────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  allegro_hand_driver
  bhand
)

ament_package()
