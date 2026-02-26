// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Author: Hibo (sh-yang@wonik.com)

#include "allegro_node.h"
#include "allegro_hand_driver/AllegroHandDrv.h"

// Joint names for JointState; must match URDF (joint_0_0 .. joint_15_0).
std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"
        };


AllegroNode::AllegroNode(const std::string nodeName, bool sim /* = false */)
  : Node(nodeName)
{
  mutex = new std::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  declare_parameter("hand_info/which_hand", "Right");
  whichHand = get_parameter("hand_info/which_hand").as_string();

  // Initialize CAN device (skip if sim == true).
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    declare_parameter("comm/CAN_CH", "can0");
    auto can_ch = this->get_parameter("comm/CAN_CH").as_string();
    if (canDevice->init(can_ch)) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  tstart = get_clock()->now();

  // Publishers: joint state, tactile (Float32MultiArray). Subscriptions: desired state, timechange, forcechange.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  tactile_sensor_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(TACTILE_SENSOR_TOPIC, 10);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(DESIRED_STATE_TOPIC, 1,
                                 std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
  time_sub = this->create_subscription<std_msgs::msg::Float32>("timechange", 1,
                                 std::bind(&AllegroNode::ControltimeCallback, this, std::placeholders::_1));
  force_sub = this->create_subscription<std_msgs::msg::Float32>("forcechange", 1,
                                 std::bind(&AllegroNode::GraspforceCallback, this, std::placeholders::_1));

}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  rclcpp::shutdown();
}

// Called when a desired joint state is received on DESIRED_STATE_TOPIC (used by base class; grasp node uses setJointCallback).
void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

// "timechange" topic: set BHand motion time (seconds).
void AllegroNode::ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    motion_time = msg->data;
    pBHand->SetMotiontime(motion_time);
}

// "forcechange" topic: set grasping force used when fingertip contact condition is met.
void AllegroNode::GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    force_get = msg->data;
}



// Publish current joint state and tactile sensor (layout: palm, then thumb→index→middle→ring root→tip).
void AllegroNode::publishData() {
  current_joint_state.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub->publish(current_joint_state);
  // tactile_sensor.data: palm, then per finger (thumb→index→middle→ring) root→tip. 16 elements.
  // [0]=palm, [1-3]=thumb(madi root,2nd, tip), [4-7]=index(root,2nd,3rd,tip), [8-11]=middle, [12-15]=ring
  tactile_sensor.data.clear();
  tactile_sensor.data.push_back(palm_sensor);
  // thumb: madi 0,1 + fingertip 0
  tactile_sensor.data.push_back(madi_sensor[0]);
  tactile_sensor.data.push_back(madi_sensor[1]);
  tactile_sensor.data.push_back(fingertip_sensor[0]);
  // index: madi 2,3,4 + fingertip 1
  tactile_sensor.data.push_back(madi_sensor[2]);
  tactile_sensor.data.push_back(madi_sensor[3]);
  tactile_sensor.data.push_back(madi_sensor[4]);
  tactile_sensor.data.push_back(fingertip_sensor[1]);
  // middle: madi 5,6,7 + fingertip 2
  tactile_sensor.data.push_back(madi_sensor[5]);
  tactile_sensor.data.push_back(madi_sensor[6]);
  tactile_sensor.data.push_back(madi_sensor[7]);
  tactile_sensor.data.push_back(fingertip_sensor[2]);
  // ring: madi 8,9,10 + fingertip 3
  tactile_sensor.data.push_back(madi_sensor[8]);
  tactile_sensor.data.push_back(madi_sensor[9]);
  tactile_sensor.data.push_back(madi_sensor[10]);
  tactile_sensor.data.push_back(fingertip_sensor[3]);
  tactile_sensor_pub->publish(tactile_sensor);
}

// Main control loop: read CAN, update positions/velocity, compute torque, write torque, publish.
void AllegroNode::updateController() {
  tnow = get_clock()->now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;

  if(dt <= 0) {
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("allegro_node"), *get_clock(), 1000, "AllegroNode::updateController dt is zero.");
    return;
  }
  tstart = tnow;

  if (canDevice)
  {
    lEmergencyStop = canDevice->readCANFrames();

    if (lEmergencyStop == 0 && canDevice->isJointInfoReady())
    {
      auto t_enter = get_clock()->now();

      // back-up previous joint positions:
      for (int i = 0; i < DOF_JOINTS; i++) {
        previous_position[i] = current_position[i];
        previous_position_filtered[i] = current_position_filtered[i];
        previous_velocity[i] = current_velocity[i];
      }

      // update joint positions:
      canDevice->getJointInfo(current_position);

      // filtering:
      for (int i = 0; i < DOF_JOINTS; i++) {
        current_position_filtered[i] = current_position[i];
        current_velocity[i] =
                (current_position[i] - previous_position[i]) / dt;
        current_velocity_filtered[i] =  current_velocity[i];
      }

      OperatingMode = 0;

      // Grasping force: if thumb+index+ring fingertip sum > threshold, use force_get; else default 2.0.
      if (OperatingMode == 0) {
        if ((fingertip_sensor[0] + fingertip_sensor[1] + fingertip_sensor[3]) > 10.0f)
          f[0] = f[1] = f[2] = force_get;
        else
          f[0] = f[1] = f[2] = 2.0f;
      }

      // Subclass computes desired_torque (e.g. BHand).
      computeDesiredTorque();

      // set & write torque to each joint:
      canDevice->setTorque(desired_torque);
      lEmergencyStop = canDevice->writeJointTorque();

      // reset joint position update flag:
      canDevice->resetJointInfoReady();

      // publish joint positions to ROS topic:
      publishData();

      frame++;
    }


    // On first successful frame, check hand handedness matches parameter (left/right).
    if (frame == 1) {
      if (whichHand.compare("left") == 0) {
        if (canDevice->RIGHT_HAND) {
          RCLCPP_ERROR(this->get_logger(), "WRONG HANDEDNESS DETECTED!");
          canDevice = 0;
          return;
        }
      } else {
        if (!canDevice->RIGHT_HAND) {
          RCLCPP_ERROR(this->get_logger(), "WRONG HANDEDNESS DETECTED!");
          canDevice = 0;
          return;
        }
      }
    }
  }

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
    rclcpp::shutdown();
  }
}

void AllegroNode::timerCallback() {
  updateController();
}

using namespace std::chrono_literals;

// Start 1 ms wall timer for control loop (used when not in polling mode).
rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(1ms, std::bind(&AllegroNode::timerCallback, this));
  return timer;
}
