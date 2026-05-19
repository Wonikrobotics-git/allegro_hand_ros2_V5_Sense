cmake_minimum_required(VERSION 3.5)
project(allegro_hand_sensor_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Executable
add_executable(allegro_hand_sensor_visualizer_node src/rviz_main.cpp)
target_link_libraries(allegro_hand_sensor_visualizer_node Qt5::Core Qt5::Widgets)
ament_target_dependencies(allegro_hand_sensor_visualizer_node
  rclcpp
  std_msgs
  visualization_msgs
  ament_index_cpp
)

# Install
install(TARGETS allegro_hand_sensor_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY asset/
  DESTINATION share/${PROJECT_NAME}/asset
)

ament_package()