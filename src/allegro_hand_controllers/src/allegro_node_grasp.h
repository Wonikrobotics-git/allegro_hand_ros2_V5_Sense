#ifndef PROJECT_ALLEGRO_NODE_GRASP_HPP
#define PROJECT_ALLEGRO_NODE_GRASP_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_node.h"

class BHand;

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasps (e.g., three-finger grasp, envelop, pinch, etc.).
// String commands (lib_cmd) are mapped to eMotionType in the .cpp file.
// Also supports PD pose from YAML (pdControl<N>, moveit, or custom pose name).
// Author: Felix Duvallet
//
class AllegroNodeGrasp : public AllegroNode {
 public:
    AllegroNodeGrasp(const std::string node_name);
    ~AllegroNodeGrasp();

    void initController(const std::string &whichHand);
    void computeDesiredTorque();

    // LIB_CMD_TOPIC: string -> grasp type or "pdControl<N>", "sensor", "moveit", or "<name>.yaml" pose.
    void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);
    // DESIRED_STATE_TOPIC: set desired joint positions and switch to eMotionType_JOINT_PD.
    void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // ENVELOP_TORQUE_TOPIC: set envelop grasp torque scalar.
    void envelopTorqueCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // Run control: polling=true -> tight loop + spin_some; false -> 1ms timer + spin.
    void doIt(bool polling);

 protected:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr envelop_torque_sub;

    BHand *pBHand = nullptr;
    double desired_position[DOF_JOINTS] = {0.0};  // Target position for PD/pose control.
};

#endif //PROJECT_ALLEGRO_NODE_GRASP_HPP
