# 🖐️ allegro_hand_ros2_V5_Sense

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange) ![Support](https://img.shields.io/badge/Support-V5%20Sense%20Only-red)

This is the official release to control **Allegro Hand V5 Sense** with ROS2. It is optimized for torque-based control and features improved interfaces and controllers developed by Soohoon Yang(Hibo) in Wonik Robotics.

> [!IMPORTANT]  
> This package **Only supports V5 Sense**. For the standard V5 model, please visit the [Allegro Hand V5 ROS2 repository](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5).

---

## ✨ Key Features
* **Real-time Visualization:** Monitor Allegro Hand V5 state in real-time using Rviz2.
* **GUI Control:** Simple hand control via a GUI tool (replaces keyboard input).
* **Gain Tuning System:** Integrated support for tuning motor gains.
* **Custom Poses:** Easily save and load custom joint positions.

---

## 📦 Packages

| Package Name | Description |
| :--- | :--- |
| **`allegro_hand_controllers`** | Main nodes for control, URDF descriptions, and meshes. |
| **`allegro_hand_driver`** | Main driver for CAN communication with the Allegro Hand. |
| **`allegro_hand_keyboard`** | Node for pre-defined keyboard commands. |
| **`allegro_hand_gui`** | GUI-based control program. |
| **`bhand`** | Library for pre-defined grasps (Default: x86-64). |

> [!TIP]
> **For ARM64 (e.g., Jetson):** If using an ARM-based system, update the symlink using [this repository](https://github.com/Wonikrobotics-git/Bhandlib_ARM).(Will be Update Soon)

---

## 🔢 Topic Description & Data Structure

### Joint Numbering Order
The `Joint Num` sequence consists of 16 joints (0~15) in the following order:

| Index Range | Finger | Sequence |
| :--- | :--- | :--- |
| `0 ~ 3` | **Index Finger** | Joint 1 (Base), 2, 3, 4 (Tip) |
| `4 ~ 7` | **Middle Finger** | Joint 1 (Base), 2, 3, 4 (Tip) |
| `8 ~ 11` | **Ring Finger** | Joint 1 (Base), 2, 3, 4 (Tip) |
| `12 ~ 15` | **Thumb** | Joint 1 (Base), 2, 3, 4 (Tip) |

### Tactile Sensor Mapping
The sensor sequence consists of 16 sensors (including the Palm):

| Index Range | Finger / Part | Sequence |
| :--- | :--- | :--- |
| `0 ~ 3` | **Thumb & Palm** | Palm, Thumb Madi 1, 2, Thumb Tip |
| `4 ~ 7` | **Index Finger** | Index Madi 1, 2, 3, Index Tip |
| `8 ~ 11` | **Middle Finger** | Middle Madi 1, 2, 3, Middle Tip |
| `12 ~ 15` | **Ring Finger** | Ring Madi 1, 2, 3, Ring Tip |

---

### Internal Data Structures

The following internal variables are used to store joint and sensor data within the `AllegroNode` and `AllegroHandDrv` classes:

| Data Type | Variable Key | Class | Description |
| :--- | :--- | :--- | :--- |
| **Joint Angles** | `current_position[16]` | `AllegroNode` | Raw joint angles (radians) received from CAN. |
| **Filtered Angles** | `current_position_filtered[16]` | `AllegroNode` | Smoothed joint angles used for control and publishing. |
| **Tactile (Palm)** | `palm_sensor` | `AllegroHandDrv` | Pressure value for the palm sensor. |
| **Tactile (Madi)** | `madi_sensor[11]` | `AllegroHandDrv` | Pressure values for the finger segments. |
| **Tactile (Tip)** | `fingertip_sensor[4]` | `AllegroHandDrv` | Pressure values for the four fingertips. |

---

## 🔍 Logging & Debugging

### 1. Command Line Logging (ROS 2 CLI)
You can monitor the joint data and sensor values directly from the terminal:

*   **To echo joint states:**
    ```bash
    ros2 topic echo /allegroHand/joint_states
    ```
*   **To echo tactile sensor data:**
    ```bash
    ros2 topic echo /allegroHand/tactile_sensors
    ```

### 2. C++ Code Logging Example
If you want to log data within the source code (e.g., for debugging in `allegro_node.cpp`), use `RCLCPP_INFO`:

```cpp
// Example: Logging specific joint angles (Joint 0 and Joint 4)
RCLCPP_INFO(this->get_logger(), "Joint 0: %.3f, Joint 4: %.3f", 
            current_position_filtered[0], current_position_filtered[4]);

// Example: Logging tactile sensor values
RCLCPP_INFO(this->get_logger(), "Palm: %.1f, Thumb Tip: %.1f", 
            palm_sensor, fingertip_sensor[0]);
```
## 🚀 Installation & Build

### 1. Prerequisites
```bash
sudo apt-get update
sudo apt-get install ros-humble-xacro ros-humble-moveit \
ros-humble-controller-manager ros-humble-joint-state-broadcaster \
ros-humble-joint-trajectory-controller
```

### 2. Workspace Setup
```bash
# Create a new workspace
mkdir -p ~/allegro_ws/src && cd ~/allegro_ws/src

# Download the ROS2 package for Allegro Hand V5 Sense
git clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_V5_Sense.git

# Build the workspace
cd ~/allegro_ws
source /opt/ros/humble/setup.bash
colcon build
```

## 🚀 Running the Controller

## 1. Launch Main Node
You can control the Allegro Hand by launching the `allegro_hand.launch.py` file.  
You must specify the handedness (right or left).

```bash
source install/setup.bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right
```
### ⚙️ Optional Arguments

You can customize the launch by adding optional arguments. By default, these are set to `false`.

| Argument | Description | Default |
| :--- | :--- | :--- |
| **`VISUALIZE:=true`** | Launch **Rviz2** for real-time hand visualization. | `false` |
| **`GUI:=true`** | Launch the **GUI Control Tool** instead of using the keyboard. | `false` |

#### Example Commands:

* **To visualize the Allegro Hand on Rviz2:**
  ```bash
  ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right VISUALIZE:=true
  ```
* **To control the Allegro HAnd with the GUI tool:**
  ```bash
  ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right GUI:=true
  ```
## 2. Run Keyboard Node
To send pre-defined commands using your keyboard:
```bash
source install/setup.bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard
```

## 🛠️ How to Use Hand Easily
### 📉 PD Gain Tuning (Fixing Vibrations or Fine Tuning)
Depending on the specific commands or motor states, the default gain values might cause vibrations during operation. 

To resolve this and optimize performance, you can tune the gains for the `joint_cmd` topic:
1. Open `allegro_node_grasp.cpp`.
2. Locate the **`kp`** (Proportional) and **`kd`** (Derivative) variables.
3. Modify the values, rebuild the package, and test to find the most stable parameters for your setup.

### 💾 Creating and Executing Custom Poses
You can quickly create, save, and execute custom hand poses by following these steps:

1. **Create a Configuration File:** Create a new `.yaml` file within the `allegro_hand_controllers` package. (Please refer to `example.yaml` for formatting rules).
2. **Define Joint Angles:** Define the target angles for all 16 joints in the specific **Joint Numbering Order** mentioned above.
3. **Build and Execute:** After creating the file and building your workspace, you can command the hand using the `lib_cmd` topic:
   * **Topic:** `allegroHand_(NUM)/lib_cmd`
   * **Command:** Send the **name of your YAML file** (without the `.yaml` extension).
   
The hand will automatically generate a smooth trajectory to reach the target pose.

### ⚠️ Important: Naming & Timing Constraints
To ensure stable and safe movement, please keep the following in mind:

* **Avoid Reserved Names:** Do **NOT** use names already reserved for default poses (e.g., `"home"`). Duplicate names may cause system conflicts.
* **Motion Duration:** The default trajectory time is set to approximately **1.2 seconds**.
* **Avoid Overlapping Commands:** If a new command is received while the hand is still moving, it may result in unintended or erratic behavior.
* **Safety Buffer:** Ensure a minimum delay of at least **1.2 seconds** between consecutive commands.

## 🤝 Control more than one hand
To control more than one hand, you must specify the **CAN port number** and the **Allegro Hand number** (`NUM`) during the launch.

#### Terminal 1: Launch Hand 0 (Right)
```bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right CAN_DEVICE:=can0 NUM:=0
```

#### Terminal 2: Launch Hand 1 (Left)
```bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=left CAN_DEVICE:=can1 NUM:=1
```
>[!NOTE]
>These are example commands. You may need to change CAN_DEVICE, PORT, and NUM arguments according to your specific hardware setup.

---
## 🔗 Useful Links
* **Official Allegro Hand Website:** [https://www.allegrohand.com/](https://www.allegrohand.com/)
* **Community Forum:** [https://www.allegrohand.com/forum](https://www.allegrohand.com/forum)
* **Allegro Hand V5 ROS1 Package:** [Wonik Robotics GitHub](https://github.com/Wonikrobotics-git/allegro_hand_ros_v5)
* **Bhandlib for ARM64:** [Bhandlib_ARM Repository](https://github.com/Wonikrobotics-git/Bhandlib_ARM)
   
