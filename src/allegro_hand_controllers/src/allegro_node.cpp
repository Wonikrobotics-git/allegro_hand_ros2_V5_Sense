// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Supports CAN or Ethernet (UDP-only) via "comm/COMM_TYPE" parameter.
//
// Author: Hibo (sh-yang@wonik.com)

#include "allegro_node.h"
#include <cstdio>
#include <unistd.h>
#include <cmath>


// frame_ids for Rviz_Arrow markers
static const std::vector<std::string> fingertip_frame_ids = {
    "link_15_0_tip", "link_3_0_tip", "link_7_0_tip", "link_11_0_tip"
};
static const std::vector<std::string> fingertip_ns = {
    "fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"
};
static const std::vector<std::string> madi_frame_ids = {
    "link_sensor_0_0", "link_sensor_1_0", "link_sensor_2_0",
    "link_sensor_3_0", "link_sensor_4_0", "link_sensor_5_0",
    "link_sensor_6_0", "link_sensor_7_0", "link_sensor_8_0",
    "link_sensor_9_0", "link_sensor_10_0"
};
static const int madi_sensor_idx[11] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 1};

// Joint names for JointState; must match URDF (joint_0_0 .. joint_15_0).
std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"
        };


AllegroNode::AllegroNode(const std::string nodeName)
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

  // Initialize communication device
  canDevice = 0;

  // Select communication type: "can" (default) or "tcp"
  declare_parameter("comm/COMM_TYPE", "can");
  auto comm_type = this->get_parameter("comm/COMM_TYPE").as_string();

  if (comm_type == "tcp") {
    /* Parameter value "tcp" selects Ethernet with UDP-only transport. */
    canDevice = new allegro::AllegroHandTcpDrv();
    declare_parameter("comm/TCP_ADDR", "192.168.0.50:7000");
    auto tcp_addr = this->get_parameter("comm/TCP_ADDR").as_string();
    RCLCPP_INFO(this->get_logger(),
        "COMM: Using UDP Ethernet (port in address is UDP peer port), %s",
        tcp_addr.c_str());
    if (canDevice->init(tcp_addr)) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  } else {
    // CAN mode (default)
    canDevice = new allegro::AllegroHandDrv();
    declare_parameter("comm/CAN_CH", "can0");
    auto can_ch = this->get_parameter("comm/CAN_CH").as_string();
    RCLCPP_INFO(this->get_logger(), "COMM: Using CAN mode, channel: %s", can_ch.c_str());
    if (canDevice->init(can_ch)) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }
  

  // Publishers: joint state, tactile (Float32MultiArray), arrow markers. Subscriptions: desired state, timechange, forcechange.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  tactile_sensor_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(TACTILE_SENSOR_TOPIC, 10);
  marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("fingertip_arrow_markers", 1);
  marker_array.markers.resize(16);  // 4 fingertip SPHERE + 11 madi ARROW + 1 palm ARROW
  // orientation w=1 초기화 — {0,0,0,0}이면 rviz2가 invalid quaternion으로 거부
  for (auto & m : marker_array.markers) {
    m.pose.orientation.w = 1.0;
  }
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(DESIRED_STATE_TOPIC, 1,
                                 std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
  time_sub = this->create_subscription<std_msgs::msg::Float32>("timechange", 1,
                                 std::bind(&AllegroNode::ControltimeCallback, this, std::placeholders::_1));
  force_sub = this->create_subscription<std_msgs::msg::Float32>("forcechange", 1,
                                 std::bind(&AllegroNode::GraspforceCallback, this, std::placeholders::_1));

  tstart = this->now();
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


// Update marker data (SPHERE for fingertips, ARROW for madi/palm).
void AllegroNode::Rviz_Arrow() {
  // stamp=0: rviz2가 가장 최신 TF를 사용 — static frame에서 stamp 불일치로 깜빡이는 것 방지
  auto now = rclcpp::Time(0);
  rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.1);

  // ── Fingertip SPHERE (× 4) ──
  for (int i = 0; i < 4; ++i) {
    float value = fingertip_sensor[i] * (255.0f / 30.0f);
    float r = 0.0f, g = 0.0f, b = 0.0f;
    if (value <= 64.0f) {
      g = value / 64.0f; b = 1.0f;
    } else if (value <= 128.0f) {
      g = 1.0f; b = 1.0f - (value - 64.0f) / 64.0f;
    } else if (value <= 192.0f) {
      r = (value - 128.0f) / 64.0f; g = 1.0f;
    } else if (value <= 255.0f) {
      r = 1.0f; g = 1.0f - (value - 192.0f) / 64.0f;
    } else {
      r = 1.0f;
    }
    float val_limit = std::min(fingertip_sensor[i], 2000.0f);
    float sz = 0.035f + val_limit / 100.0f;

    marker_array.markers[i].header.frame_id = fingertip_frame_ids[i];
    marker_array.markers[i].header.stamp    = now;
    marker_array.markers[i].lifetime        = lifetime;
    marker_array.markers[i].ns              = fingertip_ns[i];
    marker_array.markers[i].id              = i;
    marker_array.markers[i].type            = visualization_msgs::msg::Marker::SPHERE;
    marker_array.markers[i].action          = visualization_msgs::msg::Marker::ADD;
    marker_array.markers[i].scale.x = marker_array.markers[i].scale.y = marker_array.markers[i].scale.z = sz;
    marker_array.markers[i].color.r = r; marker_array.markers[i].color.g = g;
    marker_array.markers[i].color.b = b; marker_array.markers[i].color.a = 0.4f;
  }

  // ── Madi ARROW (× 11) ──
  for (int i = 0; i < 11; ++i) {
    float value = madi_sensor[madi_sensor_idx[i]];
    // 최소 1mm 보장 — tail==tip(길이0)이면 rviz2 error 발생
    float shaft_length = std::max(value / 100.0f, 0.001f);

    geometry_msgs::msg::Point p_tail, p_tip;
    p_tail.x = shaft_length; p_tail.y = 0.0; p_tail.z = 0.0;
    p_tip.x  = 0.0;          p_tip.y  = 0.0; p_tip.z  = 0.0;

    marker_array.markers[i + 4].header.frame_id = madi_frame_ids[i];
    marker_array.markers[i + 4].header.stamp    = now;
    marker_array.markers[i + 4].lifetime        = lifetime;
    marker_array.markers[i + 4].ns   = "madi_sensor_[" + std::to_string(i) + "]";
    marker_array.markers[i + 4].id   = 4 + i;
    marker_array.markers[i + 4].type = visualization_msgs::msg::Marker::ARROW;
    marker_array.markers[i + 4].action = visualization_msgs::msg::Marker::ADD;
    marker_array.markers[i + 4].pose.orientation.w = 1.0;
    marker_array.markers[i + 4].points.clear();
    marker_array.markers[i + 4].points.push_back(p_tail);
    marker_array.markers[i + 4].points.push_back(p_tip);
    marker_array.markers[i + 4].scale.x = 0.005f;
    marker_array.markers[i + 4].scale.y = 0.007f;
    marker_array.markers[i + 4].scale.z = 0.01f;
    marker_array.markers[i + 4].color.r = 1.0f; marker_array.markers[i + 4].color.g = 0.0f;
    marker_array.markers[i + 4].color.b = 0.0f; marker_array.markers[i + 4].color.a = 0.8f;
  }

  // ── Palm ARROW (× 1) ──
  {
    float shaft_length = std::max(palm_sensor / 100.0f, 0.001f);

    geometry_msgs::msg::Point p_tail, p_tip;
    p_tail.x = shaft_length; p_tail.y = 0.0; p_tail.z = 0.0;
    p_tip.x  = 0.0;          p_tip.y  = 0.0; p_tip.z  = 0.0;

    marker_array.markers[15].header.frame_id = "link_sensor_palm_0";
    marker_array.markers[15].header.stamp    = now;
    marker_array.markers[15].lifetime        = lifetime;
    marker_array.markers[15].ns     = "palm_sensor";
    marker_array.markers[15].id     = 15;
    marker_array.markers[15].type   = visualization_msgs::msg::Marker::ARROW;
    marker_array.markers[15].action = visualization_msgs::msg::Marker::ADD;
    marker_array.markers[15].pose.orientation.w = 1.0;
    marker_array.markers[15].points.clear();
    marker_array.markers[15].points.push_back(p_tail);
    marker_array.markers[15].points.push_back(p_tip);
    marker_array.markers[15].scale.x = 0.005f;
    marker_array.markers[15].scale.y = 0.007f;
    marker_array.markers[15].scale.z = 0.01f;
    marker_array.markers[15].color.r = 1.0f; marker_array.markers[15].color.g = 0.0f;
    marker_array.markers[15].color.b = 0.0f; marker_array.markers[15].color.a = 0.8f;
  }
}


// Publish current joint state and tactile sensor (layout: palm, then thumb→index→middle→ring root→tip).
void AllegroNode::publishData() {
  current_joint_state.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity_filtered[i];
    current_joint_state.effort[i] = motor_current[i];
  }
  joint_state_pub->publish(current_joint_state);
  // tactile_sensor.data: palm, then per finger (thumb→index→middle→ring) root→tip. 16 elements.
  // [0]=palm, [1-3]=thumb(madi root,2nd, tip), [4-7]=index(root,2nd,3rd,tip), [8-11]=middle, [12-15]=ring
  tactile_sensor.data.clear();
  tactile_sensor.data.push_back(palm_sensor);
  // thumb: madi 0,1 + fingertip 0
  tactile_sensor.data.push_back(madi_sensor[0]);
  tactile_sensor.data.push_back(madi_sensor[1]);
  tactile_sensor.data.push_back(fingertip_sensor[0] );
  // index: madi 2,3,4 + fingertip 1
  tactile_sensor.data.push_back(madi_sensor[2]);
  tactile_sensor.data.push_back(madi_sensor[3]);
  tactile_sensor.data.push_back(madi_sensor[4]);
  tactile_sensor.data.push_back(fingertip_sensor[1] );
  // middle: madi 5,6,7 + fingertip 2
  tactile_sensor.data.push_back(madi_sensor[5]);
  tactile_sensor.data.push_back(madi_sensor[6]);
  tactile_sensor.data.push_back(madi_sensor[7]);
  tactile_sensor.data.push_back(fingertip_sensor[2] );
  // ring: madi 8,9,10 + fingertip 3
  tactile_sensor.data.push_back(madi_sensor[8]);
  tactile_sensor.data.push_back(madi_sensor[9]);
  tactile_sensor.data.push_back(madi_sensor[10]);
  tactile_sensor.data.push_back(fingertip_sensor[3] );
  tactile_sensor_pub->publish(tactile_sensor);
  marker_pub->publish(marker_array);
}

// Main control loop: read frames, update positions/velocity, compute torque, write torque, publish.
void AllegroNode::updateController() {
  tnow = get_clock()->now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;

  if(dt <= 0) {
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("allegro_node"), *get_clock(), 1000, "AllegroNode::updateController dt is zero.");
    return;
  }

  if (canDevice)
  {
    lEmergencyStop = canDevice->readFrames();

    if (lEmergencyStop == 0 && canDevice->isJointInfoReady())
    {


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
        if ((fingertip_sensor[0] + fingertip_sensor[1] + fingertip_sensor[3]) > 3.0f)
          f[0] = f[1] = f[2] = force_get;
        else
          f[0] = f[1] = f[2] = 10.0f;
      }

      // Subclass computes desired_torque (e.g. BHand).
      computeDesiredTorque();

      // set & write torque to each joint, unless the subclass has already
      // sent the command another way (e.g. H-inf direct position mode, where
      // position and torque share wire IDs 0x180..0x18C and a trailing torque
      // write would overwrite the just-sent position target).
      if (!skipTorqueWrite()) {
        canDevice->setTorque(desired_torque);
        lEmergencyStop = canDevice->writeJointTorque();
      }

      // reset joint position update flag:
      canDevice->resetJointInfoReady();


      // update and publish Rviz arrow/sphere markers, then joint state:
      Rviz_Arrow();
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

// Start 3 ms wall timer for control loop (used when not in polling mode).
rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(3ms, std::bind(&AllegroNode::timerCallback, this));
  return timer;
}
