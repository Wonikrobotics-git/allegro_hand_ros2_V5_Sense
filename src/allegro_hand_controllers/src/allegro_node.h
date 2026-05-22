//
// Base Allegro hand ROS 2 node: CAN/TCP I/O, joint state publish, tactile publish,
// and control loop. Subclasses (e.g. AllegroNodeGrasp) implement computeDesiredTorque().
//
// Uses AllegroHandDrvBase — comm type (CAN or UDP Ethernet / “udp” param) at runtime
// via the “comm/COMM_TYPE” ROS parameter.
//
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS, global sensor variables, and allegro namespace.
#include <allegro_hand_driver/AllegroHandDrvBase.h>
#include <allegro_hand_driver/AllegroHandDrv.h>
#include <allegro_hand_driver/AllegroHandTcpDrv.h>
using namespace allegro;

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "BHand.h"
#include <vector>
#include "allegro_hand_driver/AllegroHandDrv.h"
#include "candrv/candrv.h"
#include "allegro_hand_controllers/srv/set_net_config.hpp"

// Control loop period (seconds). 333 Hz.
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";
const std::string TACTILE_SENSOR_TOPIC = "allegroHand/tactile_sensors";

class AllegroNode: public rclcpp::Node {
 public:

  AllegroNode(const std::string nodeName);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr desired);

  void ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void Rviz_Arrow();

  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"), "Called virtual function!");
  };

  // When true, updateController() will NOT call setTorque/writeJointTorque
  // after computeDesiredTorque(). Used by H-inf onboard position mode, where
  // the firmware's position wire IDs overlap with the torque wire IDs and
  // a trailing torque=0 write would overwrite the position target.
  virtual bool skipTorqueWrite() const { return false; }

  rclcpp::TimerBase::SharedPtr startTimerCallback();

  void timerCallback();

 protected:

  // Joint state. position_offset is applied when reading; filtered used for control/publish.
  double position_offset[DOF_JOINTS] = {0.0};
  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};
  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};
  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};
  double desired_torque[DOF_JOINTS] = {0.0};

  std::string whichHand;   // "Right" or "left".

  // ROS publishers & subscriptions.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tactile_sensor_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  visualization_msgs::msg::MarkerArray marker_array;  // 16: 4 fingertip + 11 madi + 1 palm
  rclcpp::Time t_last_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub;   // "timechange" -> motion_time
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub;   // "forcechange" -> force_get

  // Current and desired joint states; tactile message (layout: palm, then thumb→index→middle→ring root→tip).
  sensor_msgs::msg::JointState current_joint_state;
  sensor_msgs::msg::JointState desired_joint_state;
  std_msgs::msg::Float32MultiArray tactile_sensor;

  // ROS time and control period.
  rclcpp::Time tnow;
  rclcpp::Time tstart;
  double dt;

  // Communication device (CAN or TCP, selected by comm/COMM_TYPE parameter).
  allegro::AllegroHandDrvBase *canDevice;
  std::mutex *mutex;

  rclcpp::Service<allegro_hand_controllers::srv::SetNetConfig>::SharedPtr net_config_srv;

  // lEmergencyStop: 0 ok, <0 hand off -> shutdown. frame: control cycle count (used by BHand).
  int lEmergencyStop = 0;
  long frame = 0;

  // BHand-related: motion time (s) and grasping force; set via "timechange" and "forcechange" topics.
  double motion_time = 1.0;
  double force_get = 3.0f;
  BHand *pBHand = NULL;
};



#endif //PROJECT_ALLEGRO_NODE_COMMON_H
