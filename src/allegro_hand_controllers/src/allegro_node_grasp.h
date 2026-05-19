#ifndef PROJECT_ALLEGRO_NODE_GRASP_HPP
#define PROJECT_ALLEGRO_NODE_GRASP_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_node.h"
#include <array>

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

    // In H-inf mode we already sent a position frame from computeDesiredTorque;
    // skip the base class torque write so it doesn't clobber the same wire IDs.
    bool skipTorqueWrite() const override { return hinf_mode_; }

    // LIB_CMD_TOPIC: string -> grasp type or "pdControl<N>", "sensor", "moveit", or "<name>.yaml" pose.
    void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);
    // DESIRED_STATE_TOPIC: set desired joint positions and switch to eMotionType_JOINT_PD.
    void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // ENVELOP_TORQUE_TOPIC: set envelop grasp torque scalar.
    void envelopTorqueCallback(const std_msgs::msg::Float32::SharedPtr msg);
    // allegroHand/gain_cmd: update PD gains (kp, kd) from Float64MultiArray.
    void gainCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    // allegroHand/gff_rpy: roll, pitch, yaw in degrees for firmware gravity feed-forward.
    void gffRpyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Run control: polling=true -> tight loop + spin_some; false -> 1ms timer + spin.
    void doIt(bool polling);

 protected:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr envelop_torque_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gain_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gff_rpy_sub;

    BHand *pBHand = nullptr;
    double desired_position[DOF_JOINTS] = {0.0};  // Target position for PD/pose control.
    double gff_rpy_deg_[3] = {0.0, 0.0, 0.0};  // roll, pitch, yaw in degrees.

    // H-inf direct position control mode.
    // When true, computeDesiredTorque() sends position targets directly to
    // the firmware via sendPositionDirect() instead of using BHand torque control.
    bool hinf_mode_ = false;
    bool hinf_position_synced_ = false;
};

#endif //PROJECT_ALLEGRO_NODE_GRASP_HPP
