//
// Base Allegro hand ROS 2 node: CAN I/O, joint state publish, tactile publish,
// and control loop. Subclasses (e.g. AllegroNodeGrasp) implement computeDesiredTorque().
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS and allegro namespace (fingertip_sensor, madi_sensor, palm_sensor, etc.).
#include <allegro_hand_driver/AllegroHandDrv.h>
using namespace allegro;

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "BHand.h"
#include <vector>

// Forward declaration.
class AllegroHandDrv;

// Control loop period (seconds). 500 Hz.
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";
const std::string JOINT_TEMPERATURE_TOPIC = "allegroHand/joint_tempeartures";
const std::string TACTILE_SENSOR_TOPIC = "allegroHand/tactile_sensors";

class AllegroNode: public rclcpp::Node {
 public:

  AllegroNode(const std::string nodeName, bool sim = false);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr desired);

  void ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"), "Called virtual function!");
  };

  rclcpp::TimerBase::SharedPtr startTimerCallback();

  void timerCallback();

 protected:

  // Joint state (from CAN). position_offset is applied when reading; filtered used for control/publish.
  double position_offset[DOF_JOINTS] = {0.0};
  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};
  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};
  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};
  double desired_torque[DOF_JOINTS] = {0.0};

  int status_interval = 0;
  std::string whichHand;   // "Right" or "left".
  std::string whichType;   // non-Geared or Geared hand.

  // ROS publishers & subscriptions.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tactile_sensor_pub;
  rclcpp::Time t_last_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr joint_temperature_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub;   // "timechange" -> motion_time
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub;   // "forcechange" -> force_get
  bool joint_temperature_pending;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  // Current and desired joint states; tactile message (layout: palm, then thumb→index→middle→ring root→tip).
  sensor_msgs::msg::JointState current_joint_state;
  sensor_msgs::msg::JointState desired_joint_state;
  std_msgs::msg::UInt8MultiArray current_joint_temperature;
  std_msgs::msg::Float32MultiArray tactile_sensor;

  // ROS time and control period.
  rclcpp::Time tstart;
  rclcpp::Time tnow;
  rclcpp::Time t_last_joint_info_ready;  // last time isJointInfoReady() was true (for cycle logging)
  double dt;

  // CAN device and lock for desired_joint_state.
  allegro::AllegroHandDrv *canDevice;
  std::mutex *mutex;

  // lEmergencyStop: 0 ok, <0 hand off -> shutdown. frame: control cycle count (used by BHand).
  int lEmergencyStop = 0;
  long frame = 0;

  // BHand-related: motion time (s) and grasping force; set via "timechange" and "forcechange" topics.
  double motion_time = 1.0;
  double force_get = 3.0f;
  BHand *pBHand = NULL;
};



#endif //PROJECT_ALLEGRO_NODE_COMMON_H
