#include "allegro_node_grasp.h"
#include <iostream>
#include <fstream>
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";
const std::string GFF_RPY_TOPIC = "allegroHand/gff_rpy";

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME},   // home position
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
};

//Define PD gain for eMotionType_JOINT_PD
double kp[DOF_JOINTS] = { /// default p_gain
  ///root -> tip
  1.0,3.0,0.6,0.6, // index   
  1.0,3.0,0.6,0.6, // middle
  1.0,3.0,0.6,0.6, // pinky
  0.6,0.6,0.8,0.6  // thumb
};

double kd[DOF_JOINTS] = { /// default d_gain
  ///root -> tip
  0.05,0.15,0.03,0.04, // index
  0.05,0.15,0.03,0.04, // middle
  0.05,0.15,0.03,0.04, // pinky
  0.04,0.04,0.04,0.035  // thumb
};


AllegroNodeGrasp::AllegroNodeGrasp(const std::string nodeName)
        : AllegroNode(nodeName),pBHand(nullptr) {

  // H-inf direct position control mode (default: false).
  // Set via launch argument HINF_MODE=true or ros2 param set ... hinf_mode true.
  declare_parameter<bool>("hinf_mode", false);
  hinf_mode_ = get_parameter("hinf_mode").as_bool();
  initController(whichHand);
  
  if (hinf_mode_) {
    RCLCPP_INFO(this->get_logger(), "CTRL: H-inf direct position mode INITIALIZING (after init controller)");
    hinf_position_synced_ = false;
    if (canDevice) {
      canDevice->setHinfMode(true);
      usleep(50000);  // 50ms: wait for firmware mode switch before control loop starts
    }
  }
  else
  {
    if (canDevice) {
      canDevice->setHinfMode(false);
    }
  }


  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
          DESIRED_STATE_TOPIC, 3, std::bind(&AllegroNodeGrasp::setJointCallback, this, std::placeholders::_1));
  lib_cmd_sub = this->create_subscription<std_msgs::msg::String>(
          LIB_CMD_TOPIC, 1, std::bind(&AllegroNodeGrasp::libCmdCallback, this, std::placeholders::_1));
  envelop_torque_sub = this->create_subscription<std_msgs::msg::Float32>(
          ENVELOP_TORQUE_TOPIC, 1, std::bind(&AllegroNodeGrasp::envelopTorqueCallback, this, std::placeholders::_1));
  gain_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "allegroHand/gain_cmd", 1, std::bind(&AllegroNodeGrasp::gainCmdCallback, this, std::placeholders::_1));
  gff_rpy_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          GFF_RPY_TOPIC, 1, std::bind(&AllegroNodeGrasp::gffRpyCallback, this, std::placeholders::_1));
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand;
}

void AllegroNodeGrasp::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data.c_str();

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand->SetMotionType(itr->second);
    RCLCPP_INFO(this->get_logger(), "motion type = %d", itr->second);

    // If 'home' is commanded and H-inf mode is active, sync the firmware mode as well.
    if (lib_cmd == "home" && hinf_mode_) {
      if (canDevice) {
        canDevice->setHinfMode(true);
      }
    }

 }  else if (lib_cmd.compare("hinf_on") == 0) {
    // Switch to H-inf onboard position control mode.
    // The firmware must already be in H-inf mode (enabled via separate channel / FW config).
    // From this point computeDesiredTorque() sends position targets directly
    // via sendPositionDirect() rather than BHand torque commands.
    hinf_mode_ = true;
    hinf_position_synced_ = false;
    pBHand->SetMotionType(eMotionType_NONE);  // stop BHand from writing torque
    if (canDevice) {
      canDevice->setHinfMode(true);
    }
    RCLCPP_INFO(this->get_logger(), "CTRL: H-inf direct position mode ON");

  } else if (lib_cmd.compare("hinf_off") == 0) {
    // Return to normal BHand torque control.
    hinf_mode_ = false;
    hinf_position_synced_ = false;
    if (canDevice) {
      canDevice->setHinfMode(false);
    }
    RCLCPP_INFO(this->get_logger(), "CTRL: H-inf direct position mode OFF");

  } else if(lib_cmd.compare("calibration") == 0) {
    ///motor calibration
    //caution! : All motor encoder angle values will be set to 0.(Joint 13 will be set to 90 degrees)
    canDevice->calibrate();

  } 
  else {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
    std::string file_path = pkg_path + "/pose/" + lib_cmd + ".yaml";

    std::ifstream infile(file_path);
    if (!infile.good()) {
      RCLCPP_WARN(this->get_logger(), "Pose file does not exist. Please select a different command.");
      return;
    }
    YAML::Node node = YAML::LoadFile(file_path);
    std::vector<double> positions = node["position"].as<std::vector<double>>();

    if ((int)positions.size() < DOF_JOINTS) {
      RCLCPP_WARN(this->get_logger(), "Pose file has insufficient joint positions (%zu < %d).",
                  positions.size(), DOF_JOINTS);
      return;
    }

    for (int i = 0; i < DOF_JOINTS; i++) {
        desired_position[i] = positions[i];
    }

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_POSE_PD);
    hinf_position_synced_ = true;
  }
}

// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg->position[i];
  mutex->unlock();

  pBHand->SetJointDesiredPosition(desired_position);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
  pBHand->SetGainsEx(kp,kd);
  hinf_position_synced_ = true;
}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  
  const double torque = msg->data;
  RCLCPP_INFO(this->get_logger(), "Setting envelop torque to %.3f.", torque);
  pBHand->SetEnvelopTorqueScalar(torque);
  
}

void AllegroNodeGrasp::computeDesiredTorque() {

  if (hinf_mode_) {
    if (!hinf_position_synced_) {
      for (int i = 0; i < DOF_JOINTS; i++) {
        desired_position[i] = current_position_filtered[i];
      }
      hinf_position_synced_ = true;
      RCLCPP_INFO(this->get_logger(),
          "CTRL: H-inf desired position synced to current joint position");
    }
    // H-inf onboard position control: send desired angles directly to firmware.
    // The firmware's H-inf controller closes the loop; we do not send torque.
    if (canDevice) {
      canDevice->sendGravityFeedForwardRPY(gff_rpy_deg_[0],
                                           gff_rpy_deg_[1],
                                           gff_rpy_deg_[2]);
      canDevice->sendPositionDirect(desired_position);
    }
    // Zero out torque so the base class writeJointTorque() does nothing harmful.
    for (int i = 0; i < DOF_JOINTS; i++) desired_torque[i] = 0.0;
    return;
  }

  // Normal BHand torque control path.
  pBHand->SetJointPosition(current_position_filtered);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl(static_cast<double>(frame) * ALLEGRO_CONTROL_TIME_INTERVAL);

  // Set grasping force and get FK result (if needed)
  pBHand->SetGraspingForce(f);
  pBHand->GetFKResult(x, y, z);

  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

}

void AllegroNodeGrasp::gainCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if ((int)msg->data.size() >= 2 * DOF_JOINTS) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      kp[i] = msg->data[i];
      kd[i] = msg->data[DOF_JOINTS + i];
    }
  }
}

void AllegroNodeGrasp::gffRpyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if ((int)msg->data.size() < 3) {
    RCLCPP_WARN(this->get_logger(),
        "CTRL: gff_rpy expects [roll_deg, pitch_deg, yaw_deg]");
    return;
  }

  gff_rpy_deg_[0] = msg->data[0];
  gff_rpy_deg_[1] = msg->data[1];
  gff_rpy_deg_[2] = msg->data[2];
}

void AllegroNodeGrasp::initController(const std::string &whichHand) {

  // Load gains from package config/gain.yaml
  try {
    std::string gain_file = ament_index_cpp::get_package_share_directory("allegro_hand_controllers")
                            + "/config/gain.yaml";
    std::ifstream infile(gain_file);
    if (infile.good()) {
      YAML::Node node = YAML::LoadFile(gain_file);
      auto kp_vals = node["kp"].as<std::vector<double>>();
      auto kd_vals = node["kd"].as<std::vector<double>>();
      for (int i = 0; i < DOF_JOINTS && i < (int)kp_vals.size(); i++)
        kp[i] = kp_vals[i];
      for (int i = 0; i < DOF_JOINTS && i < (int)kd_vals.size(); i++)
        kd[i] = kd_vals[i];
      RCLCPP_INFO(this->get_logger(), "CTRL: Loaded PD gains from %s", gain_file.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "CTRL: gain.yaml not found at %s (using defaults)", gain_file.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "CTRL: Failed to load gain YAML: %s (using defaults)", e.what());
  }
  
  // Initialize BHand controller
  if (whichHand == "left") {
    pBHand = new BHand(eHandType_Left);
    RCLCPP_WARN(this->get_logger(), "CTRL: Left Allegro Hand controller initialized.");
  } else {
    pBHand = new BHand(eHandType_Right);
    RCLCPP_WARN(this->get_logger(), "CTRL: Right Allegro Hand controller initialized.");
  }

  pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = current_position[i];

  RCLCPP_INFO(this->get_logger(), "*************************************");
  RCLCPP_INFO(this->get_logger(), "         Grasp (BHand) Method        ");
  RCLCPP_INFO(this->get_logger(), "-------------------------------------");
  RCLCPP_INFO(this->get_logger(), "         Every command works.        ");
  RCLCPP_INFO(this->get_logger(), "*************************************");
}

void AllegroNodeGrasp::doIt(bool polling) {
  auto this_node = std::shared_ptr<AllegroNodeGrasp>(this);
  if (polling) {
    RCLCPP_INFO(this->get_logger(), " Polling = true (loop runs as fast as possible).");
    while (rclcpp::ok()) {
      updateController();
      rclcpp::spin_some(this_node);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Polling = false.");
    rclcpp::TimerBase::SharedPtr timer = startTimerCallback();
    rclcpp::spin(this_node);
  }
}

int main(int argc, char **argv) {
  // Set SCHED_FIFO real-time priority (requires cap_sys_nice)
  struct sched_param sp;
  sp.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
    fprintf(stderr, "Warning: Failed to set SCHED_FIFO (run setcap cap_sys_nice+ep on this executable)\n");
  }

  auto clean_argv = rclcpp::init_and_remove_ros_arguments(argc, argv);
  setvbuf(stdout, NULL, _IONBF, 0);
  bool polling = false;
  if (clean_argv.size() > 1 && clean_argv[1] == std::string("true")) {
    polling = true;
  }
  AllegroNodeGrasp allegroNode("allegro_node_grasp");
  allegroNode.doIt(polling);
}
