cmake_minimum_required(VERSION 3.8)
project(allegro_hand_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########

# Enable -fPIC so the static library can be linked into shared libraries
# (required for ros2_control hardware interface plugins)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# include: for allegro_hand_driver/*.h | src: for candrv/candrv.h (used by AllegroHandDrv.cpp and dependents)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(allegro_hand_driver
  src/AllegroHandDrv.cpp
  src/candrv/socket_can.cpp
)

ament_target_dependencies(allegro_hand_driver
  rclcpp
  std_msgs)
#############
## Install ##
#############

install(TARGETS allegro_hand_driver
        EXPORT allegro_hand_driver_export
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

## Install candrv headers so dependents (e.g. allegro_hand_controllers) can #include "candrv/candrv.h"
install(
  DIRECTORY src/candrv/
  DESTINATION include/candrv
  FILES_MATCHING PATTERN "*.h")

ament_export_targets(allegro_hand_driver_export HAS_LIBRARY_TARGET)
## Export include + src so dependents find both allegro_hand_driver/*.h and candrv/*.h (src needed when building from source)
ament_export_include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_export_dependencies(std_msgs)

ament_package()
