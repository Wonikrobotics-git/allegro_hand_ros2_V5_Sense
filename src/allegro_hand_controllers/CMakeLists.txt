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
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(allegro_hand_driver REQUIRED)
find_package(bhand REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetNetConfig.srv"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${allegro_hand_driver_INCLUDE_DIRS}
  ${bhand_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# ─────────────────────────────────────
# Build (CAN + UDP Ethernet via AllegroHandDrvBase)
# ─────────────────────────────────────
# Use an OBJECT lib (not STATIC) so Jazzy/GNU ld --as-needed does not drop ROS
# shared libs (e.g. libsensor_msgs__rosidl_generator_c.so) that are only
# referenced from allegro_node.o — see linker DT_NEEDED ordering with .a archives.
add_library(allegro_node OBJECT src/allegro_node.cpp)
add_dependencies(allegro_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_target_dependencies(
  allegro_node rclcpp sensor_msgs std_msgs visualization_msgs geometry_msgs allegro_hand_driver ament_index_cpp
)
# Ethernet (UDP) driver is a separate static lib; link it on the final executable.
get_filename_component(_allegro_pkg_prefix "${allegro_hand_driver_DIR}" DIRECTORY)
get_filename_component(_allegro_pkg_prefix "${_allegro_pkg_prefix}" DIRECTORY)
get_filename_component(_allegro_pkg_prefix "${_allegro_pkg_prefix}" DIRECTORY)

add_executable(allegro_node_grasp
  src/allegro_node_grasp.cpp
  $<TARGET_OBJECTS:allegro_node>
)
# Plain target_link_libraries (no PRIVATE/PUBLIC): ament_target_dependencies uses
# the plain signature; mixing keyword + plain on the same target breaks CMake.
target_link_libraries(allegro_node_grasp
  "${_allegro_pkg_prefix}/lib/liballegro_hand_tcp_driver.a"
  yaml-cpp
  ${bhand_LIBRARIES}
  ament_index_cpp::ament_index_cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(allegro_node_grasp rclcpp sensor_msgs std_msgs visualization_msgs geometry_msgs allegro_hand_driver yaml-cpp bhand ament_index_cpp)

# ─────────────────────────────────────
# Install
# ─────────────────────────────────────

install(TARGETS
  allegro_node_grasp
  DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
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
