# 🖐️ allegro_hand_ros2_V5_Sense

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-orange) ![Support](https://img.shields.io/badge/Support-V5%20Sense%20Only-red) ![Comm](https://img.shields.io/badge/Communication-CAN%20%7C%20Ethernet-green)

This is the official release to control **Allegro Hand V5 Sense** with ROS2. It is optimized for torque-based control and features improved interfaces and controllers developed by Soohoon Yang(Hibo) in Wonik Robotics.

> [!IMPORTANT]  
> This package **Only supports V5 Sense**. For the V5 Plus model, please visit the [Allegro Hand V5 ROS2 repository](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5).

---

## ✨ Key Features
* **Real-time Visualization:** Monitor Allegro Hand V5 Sense state in real-time using Rviz2.
* **GUI Control:** Simple hand control via a GUI tool (replaces keyboard input).
* **Gain Tuning System:** GUI-based real-time PD gain tuning with YAML config support.
* **Custom Poses:** Easily save and load custom joint positions.
* **Motor Current Feedback:** Motor current (mA) per joint is available via `joint_states.effort` in Ethernet (UDP) mode.
* **Device Configuration Panel:** GUI tool for motor calibration (Ethernet mode).
* **H-inf Onboard Position Control:** Firmware-side H∞ controller mode; PC sends position targets instead of torque commands.
* **Gravity Feed-Forward:** Send hand Roll/Pitch/Yaw to the firmware so the onboard gravity compensation activates automatically.
* **Tactile Sensor Visualizer:** Qt-based GUI for real-time display of all 16 tactile sensor values (kPa).
* **333 Hz Control Loop:** Real-time control at 3 ms intervals with `SCHED_FIFO` priority scheduling.

---

## 🚀 Installation & Build

### 1. Prerequisites

> [!NOTE]
> **ROS 2 Humble (Ubuntu 22.04)** — CAN and Ethernet both supported.  
> **ROS 2 Jazzy (Ubuntu 24.04)** — CAN supported. Ethernet mode is under development.

```bash
sudo apt-get update
# Replace 'humble' with 'jazzy' if using Ubuntu 24.04
sudo apt-get install ros-humble-xacro \
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

**Communication:** Choose **CAN** (default) or **Ethernet (UDP)** when you launch. The same `allegro_node_grasp` binary is used; only the transport changes.

### 🌐 PC Network Setup (Ethernet mode only)

> [!WARNING]
> Ethernet mode is currently supported on **ROS 2 Humble (Ubuntu 22.04)** only. Support for Jazzy (Ubuntu 24.04) is under development.

Before launching with `CAN:=false`, set the PC's network interface (connected to the hand) to a **static IPv4 address in the `192.168.0.x` subnet**.

The hand's default IP is **`192.168.0.50`** (port **`7000`**), so the PC address must be in the same subnet but different from the hand's IP (e.g. `192.168.0.100`).

**Ubuntu Settings → Network → (your wired interface) → ⚙ → IPv4**

| Field | Value |
| :--- | :--- |
| **Method** | Manual |
| **Address** | `192.168.0.100` (or any free address in `192.168.0.x`) |
| **Netmask** | `255.255.255.0` |
| **Gateway** | (leave blank) |

After applying, verify connectivity:
```bash
ping 192.168.0.50
```

> [!NOTE]
> The hand IP and port can be changed in the firmware. If your hand uses a different address, adjust `TCP_ADDR` in the launch command accordingly.

* **CAN (`CAN:=true`, default)** — Hand on SocketCAN (e.g. `can0`). The launch file runs the usual CAN setup (bitrate, `setcap`, etc.) automatically.
* **Ethernet (`CAN:=false`)** — **UDP-only** stack on port 7000. PC sends commands (HELLO, TORQUE_CMD, etc.) and receives TELEMETRY_RAW at ~**3 ms** (encoders, currents, sensors, velocity) all on the same UDP socket.

```bash
source install/setup.bash

# CAN (default)
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right

# CAN with a specific interface
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right CAN:=true CAN_DEVICE:=can0

# Ethernet (UDP, default peer address 192.168.0.50:7000)
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right CAN:=false TCP_ADDR:=192.168.0.50:7000
```

If `CAN:=false`, the launch file **skips** interactive CAN bring-up.

The control loop runs **read telemetry → compute torque → send torque** each cycle. In Ethernet mode, joint/sensor data come from UDP telemetry at ~**3 ms** cadence. In CAN mode, data follow the CAN driver timing.

### ⚙️ Optional Arguments

You can customize the launch by adding optional arguments. By default, these are set to `false`.

| Argument | Description | Default |
| :--- | :--- | :--- |
| **`VISUALIZE:=true`** | Launch **Rviz2** for real-time hand visualization. | `false` |
| **`GUI:=true`** | Launch the **GUI Control Tool** instead of using the keyboard. | `false` |
| **`HINF_MODE:=true`** | Enable **H-inf onboard position control** mode. Firmware must already be configured for H-inf. | `false` |
| **`DEMO:=true`** | Launch the **Sensor Visualizer GUI** for real-time tactile sensor display (no Rviz2). | `false` |

#### Example Commands:

* **To visualize the Allegro Hand on Rviz2:**
  ```bash
  ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right VISUALIZE:=true
  ```
* **To control the Allegro Hand with the GUI tool:**
  ```bash
  ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right GUI:=true
  ```
* **To launch the Sensor Visualizer (DEMO mode):**
  ```bash
  ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right DEMO:=true
  ```

## 2. Run Keyboard Node
To send pre-defined commands using your keyboard:
```bash
source install/setup.bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard
```

#### Keyboard Key Mapping
| Key | Command | Description |
| :--- | :--- | :--- |
| `H` | `home` | Home Pose |
| `P` | `pinch_it` | Pinch (index + thumb) |
| `M` | `pinch_mt` | Pinch (middle + thumb) |
| `G` | `grasp_3` | Grasp (3 fingers) |
| `K` | `grasp_4` | Grasp (4 fingers) |
| `E` | `envelop` | Envelop grasp |
| `A` | `gravcomp` | Gravity compensation |
| `C` | `calibration` | Motor calibration |
| `F` | `off` | Motors Off (free motion) |
| `?` or `/` | — | Print usage |

> [!WARNING]
> The keyboard node publishes to `allegroHand_0/lib_cmd` by default. If you are using a different hand index (`NUM`), you need to remap the topic accordingly.

## 3. Control more than one hand
To control more than one hand, specify the **CAN port** (or **`CAN:=false TCP_ADDR:=<IP>`** for Ethernet/UDP) and the **Allegro Hand number** (`NUM`) per launch.

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

#### Topic Remapping Rule
All topics are automatically remapped with the `allegroHand_<NUM>/` prefix based on the `NUM` argument:

| `NUM` | Topic Prefix | Example |
| :--- | :--- | :--- |
| `0` (default) | `allegroHand_0/` | `allegroHand_0/joint_states`, `allegroHand_0/lib_cmd` |
| `1` | `allegroHand_1/` | `allegroHand_1/joint_states`, `allegroHand_1/lib_cmd` |

#### Standalone Rviz2
You can also launch Rviz2 separately (e.g., to visualize a hand that is already running):
```bash
ros2 launch allegro_hand_controllers allegro_viz.launch.py NUM:=0
```

---

## 📦 Packages

| Package Name | Description |
| :--- | :--- |
| **`allegro_hand_controllers`** | Main nodes for control, URDF descriptions, and meshes. |
| **`allegro_hand_driver`** | Main driver for CAN and Ethernet (UDP) communication with the Allegro Hand. |
| **`allegro_hand_keyboards`** | Node for pre-defined keyboard commands. |
| **`allegro_hand_gui`** | GUI-based control program. |
| **`allegro_hand_sensor_visualizer`** | Qt GUI node for real-time tactile sensor visualization (launched with `DEMO:=true`). Displays fingertip (4), madi (11), and palm (1) sensor values in kPa. |
| **`bhand`** | Library for pre-defined grasps. (x86_64 & arm64 all supported)|

---

## 🔢 Topic Description & Data Structure

### ROS 2 Topics

#### 📁 Published Topics
| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| **`allegroHand/joint_states`** | `sensor_msgs/JointState` | Current joint positions (rad), velocities, and motor currents (mA) for all 16 joints. |
| **`allegroHand/tactile_sensors`** | `std_msgs/Float32MultiArray` | Data from tactile sensors (Palm, thumb, index, middle, ring). |
| **`allegroHand/fingertip_arrow_markers`** | `visualization_msgs/MarkerArray` | Rviz2 visualization markers for tactile sensors (4 fingertip spheres + 11 madi arrows + 1 palm arrow). Used with `VISUALIZE:=true`. |

> [!NOTE]
> In Ethernet (UDP) mode (`CAN:=false`), `joint_states.effort[i]` carries the **measured motor current (mA)** for joint `i` as received from the UDP telemetry stream. In CAN mode, effort values are not populated.

#### 📁 Subscribed Topics
| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| **`allegroHand/joint_cmd`** | `sensor_msgs/JointState` | Target joint positions for PD control. |
| **`allegroHand/lib_cmd`** | `std_msgs/String` | Pre-defined grasp commands (see [lib_cmd Command Reference](#-libcmd-command-reference) below). |
| **`allegroHand/gain_cmd`** | `std_msgs/Float64MultiArray` | PD gain update (first 16: Kp, next 16: Kd). Published by Gain Tuning Panel. |
| **`allegroHand/envelop_torque`** | `std_msgs/Float32` | Set torque value for envelop grasping.(0.0 ~ 1.0) |
| **`allegroHand_<NUM>/time_chg`** | `std_msgs/Float32` | Set motion duration for pre-defined movements (seconds). (internal topic: `timechange`, remapped by launch) |
| **`allegroHand_<NUM>/force_chg`** | `std_msgs/Float32` | Set grasping force used for fingertip contact routines. (internal topic: `forcechange`, remapped by launch) |
| **`allegroHand/gff_rpy`** | `std_msgs/Float64MultiArray` | Roll, Pitch, Yaw in degrees `[roll, pitch, yaw]` for firmware gravity feed-forward. Active only in H-inf mode. |

### 📋 `lib_cmd` Command Reference

The following commands can be sent via the `allegroHand/lib_cmd` topic:

| Command | Action |
| :--- | :--- |
| `home` | Move to home position |
| `grasp_3` | 3-finger grasp |
| `grasp_4` | 4-finger grasp |
| `pinch_it` | Pinch (index + thumb) |
| `pinch_mt` | Pinch (middle + thumb) |
| `envelop` | Envelop grasp (power grasp) |
| `off` | Motors off (free motion) |
| `gravcomp` | Gravity compensation mode |
| `pdControl<N>` | Load `pose/pose<N>.yaml` → PD control |
| `moveit` | Load `pose/pose_moveit.yaml` → PD control |
| `hinf_on` | Enable H-inf onboard position control |
| `hinf_off` | Disable H-inf (return to BHand torque control) |
| `calibration` | Motor encoder calibration |
| `<custom_name>` | Load `pose/<custom_name>.yaml` → PD control |

Example (default `NUM:=0`):
```bash
ros2 topic pub --once /allegroHand_0/lib_cmd std_msgs/msg/String "{data: grasp_3}"
```

> [!NOTE]
> All topic examples below use the default `NUM:=0` prefix (`allegroHand_0/`). Replace with `allegroHand_<NUM>/` if you launched with a different `NUM`.

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

### Internal Data Structures

The following internal variables are used to store joint and sensor data within the `AllegroNode` and driver classes:

| Data Type | Variable Key | Class | Description |
| :--- | :--- | :--- | :--- |
| **Joint Angles** | `current_position[16]` | `AllegroNode` | Raw joint angles (radians) received from CAN/UDP. |
| **Filtered Angles** | `current_position_filtered[16]` | `AllegroNode` | Smoothed joint angles used for control and publishing. |
| **Tactile (Palm)** | `palm_sensor` | `AllegroHandDrv` / `AllegroHandTcpDrv` | Pressure value for the palm sensor. |
| **Tactile (Madi)** | `madi_sensor[11]` | `AllegroHandDrv` / `AllegroHandTcpDrv` | Pressure values for the finger segments. |
| **Tactile (Tip)** | `fingertip_sensor[4]` | `AllegroHandDrv` / `AllegroHandTcpDrv` | Pressure values for the four fingertips. |
| **Motor Current** | `motor_current[16]` | `AllegroHandTcpDrv` | Measured motor current per joint (mA), Ethernet (UDP) mode only. Remapped to CAN joint order. |

---

## 🔍 Logging & Debugging

### 1. Command Line Logging (ROS 2 CLI)
You can monitor the joint data and sensor values directly from the terminal:

*   **To echo joint states:**
    ```bash
    ros2 topic echo /allegroHand_0/joint_states
    ```
*   **To echo tactile sensor data:**
    ```bash
    ros2 topic echo /allegroHand_0/tactile_sensors
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

---


## 🛠️ How to Use Hand Easily
### 📉 PD Gain Tuning (Fixing Vibrations or Fine Tuning)
Depending on the specific commands or motor states, the default gain values might cause vibrations during operation.

PD gains can be tuned in real-time using the **Gain Tuning Panel** without rebuilding the package.

#### Using the Gain Tuning Panel (GUI)
1. Launch the controller with the GUI enabled:
   ```bash
   ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right GUI:=true
   ```
2. In the GUI menu bar, click **Tools → Gain Tuning Panel**.
3. The panel opens with two sections:
   - **Left side** — Kp / Kd vertical sliders grouped by finger (Index, Middle, Ring, Thumb).
   - **Right side** — Joint position horizontal sliders for all 16 joints.
4. Adjust values using sliders or by typing directly into the spin boxes.
   - **Position changes** are published in real-time via `allegroHand/joint_cmd`.
   - **Gain changes** are published in real-time via `allegroHand/gain_cmd`.
   - Position and gain topics are independent — changing one does not affect the other.
5. When the panel opens, the current joint positions are automatically loaded from `allegroHand/joint_states`.

#### Saving & Loading Gains
- Click **Save YAML** to persist the current Kp/Kd values to `config/gain.yaml`.
- Click **Load YAML** to restore gains from the saved file.
- On startup, the controller automatically loads gains from `config/gain.yaml` if available. Otherwise, the default values defined in `allegro_node_grasp.cpp` are used.

#### Gain Configuration File
The gain values are stored in `allegro_hand_controllers/config/gain.yaml`:
```yaml
kp:
  - 1   # J0  index root
  - 3   # J1  index
  # ... (16 values total)
kd:
  - 0.05  # J0  index root
  - 0.15  # J1  index
  # ... (16 values total)
```

#### Default PD Gain Values
When `config/gain.yaml` is not present, the following default values are used:

| Joint | Finger | Kp | Kd |
| :--- | :--- | :--- | :--- |
| J0 | Index (root) | 1.0 | 0.05 |
| J1 | Index | 3.0 | 0.15 |
| J2 | Index | 0.6 | 0.03 |
| J3 | Index (tip) | 0.6 | 0.04 |
| J4 | Middle (root) | 1.0 | 0.05 |
| J5 | Middle | 3.0 | 0.15 |
| J6 | Middle | 0.6 | 0.03 |
| J7 | Middle (tip) | 0.6 | 0.04 |
| J8 | Ring (root) | 1.0 | 0.05 |
| J9 | Ring | 3.0 | 0.15 |
| J10 | Ring | 0.6 | 0.03 |
| J11 | Ring (tip) | 0.6 | 0.04 |
| J12 | Thumb (root) | 0.6 | 0.04 |
| J13 | Thumb | 0.6 | 0.04 |
| J14 | Thumb | 0.8 | 0.04 |
| J15 | Thumb (tip) | 0.6 | 0.035 |

### 💾 Creating and Executing Custom Poses

#### Using the GUI (Recommended)
1. Launch the controller with the GUI enabled.
2. Move the hand to the desired pose (using the pre-defined pose buttons or the **Gain Tuning Panel** sliders).
3. In the **Save Pose** section, type a name (without extension) and click **Save**.
   - The GUI reads the current `allegroHand/joint_states` and saves all 16 joint positions to `pose/<name>.yaml` inside the `allegro_hand_controllers` package.
4. The saved pose appears in the **Load Pose** list automatically.
   - Select a pose from the list and click **Load** to execute it.
   - Click **Refresh** to update the list if files were added externally.

#### Using the Command Line
You can also manually execute custom poses inside `allegro_hand_controllers/pose/` by sending its name (without extension) via the `lib_cmd` topic:
```bash
ros2 topic pub /allegroHand_0/lib_cmd std_msgs/String "data: 'my_pose'"
```
The controller will load `pose/my_pose.yaml` and move the hand to the saved joint positions. Refer to `example.yaml` for the required file format.

#### 🔁 Running a Pose Sequence (GUI)
The GUI supports running saved poses in sequence automatically:
1. Set the number of poses to sequence using the **Pose Count** spinner and click **Set**.
2. A pose list appears — click each pose in the desired execution order and click **Select** for each.
3. Once all poses are selected, set the **Repeat Count** and click **Start Sequence**.
   - Poses are executed one by one, spaced by the current motion time.
4. Click **Reset** to clear the selection and start over.

### ⚠️ Important: Naming & Timing Constraints
To ensure stable and safe movement, please keep the following in mind:

* **Avoid Reserved Names:** Do **NOT** use names already reserved for default poses (e.g., `"home"`). Duplicate names may cause system conflicts.
* **Motion Duration:** The default trajectory time is set to approximately **1.0 seconds**.
* **Avoid Overlapping Commands:** If a new command is received while the hand is still moving, it may result in unintended or erratic behavior.
* **Safety Buffer:** Ensure a minimum delay of at least **1.0 seconds** between consecutive commands.

---

## 🤖 H-inf Onboard Position Control

In H-inf mode the firmware runs an H∞ controller internally. The PC sends **desired joint angles** instead of torques; the firmware closes the control loop itself.

### Enabling H-inf Mode

Launch with `HINF_MODE:=true` to activate H-inf from the start:
```bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right HINF_MODE:=true
```

You can also toggle H-inf at runtime by publishing on `allegroHand/lib_cmd`:
```bash
# Turn H-inf ON
ros2 topic pub --once /allegroHand_0/lib_cmd std_msgs/msg/String "{data: hinf_on}"

# Turn H-inf OFF (return to normal BHand torque control)
ros2 topic pub --once /allegroHand_0/lib_cmd std_msgs/msg/String "{data: hinf_off}"
```

### How It Works

| Normal mode | H-inf mode |
| :--- | :--- |
| PC computes torque (BHand library) and sends torque commands | PC sends position targets; firmware H∞ controller computes torque internally |
| `allegroHand/joint_cmd` used for PD tracking | Same topic used; targets forwarded directly to firmware |

> [!NOTE]
> When H-inf mode is first activated, desired positions are automatically synced to the current joint positions to prevent sudden motion.

### Communication Protocol

| Transport | Command | Details |
| :--- | :--- | :--- |
| **CAN** | CAN ID `0x228` | `data[0]`: `0` = OFF, `1` = ON |
| **Ethernet (UDP)** | Packet `type = 0x60` | `payload[0]`: `0` = OFF, `1` = ON. Fire-and-forget (no ACK). |

---

## 🌐 Gravity Feed-Forward (RPY)

When the hand is mounted on a robot arm, the wrist orientation changes the effective gravity vector seen by each finger joint. Sending the current Roll/Pitch/Yaw of the hand to the firmware lets the onboard gravity compensator adjust torque offsets automatically.

> [!NOTE]
> Gravity feed-forward is active only when **H-inf mode is enabled**.

### Sending RPY

Publish a 3-element `Float64MultiArray` (degrees) on `allegroHand/gff_rpy`:
```bash
# Example: roll=10.5°, pitch=-5.25°, yaw=90.0°
ros2 topic pub /allegroHand_0/gff_rpy std_msgs/msg/Float64MultiArray \
  "{data: [10.5, -5.25, 90.0]}"
```

### Communication Protocol

| Transport | Command | Details |
| :--- | :--- | :--- |
| **CAN** | CAN ID `0x22C` | `data[0..1]` = roll, `data[2..3]` = pitch, `data[4..5]` = yaw (int16, centi-degrees, little-endian) |
| **Ethernet (UDP)** | Packet `type = 0x61` | Payload: 6 bytes — `int16` roll\_cdeg, pitch\_cdeg, yaw\_cdeg (little-endian) |

---

## 🔧 Device Configuration Panel (GUI)

> [!CAUTION]
> **This panel is intended for factory setup and emergency recovery only.**
> Motor Calibration makes **irreversible changes to firmware state**.
> Incorrect use can result in wrong joint angle references.
> **Do not use this panel during normal operation.**

The **Device Configuration** panel provides motor calibration. Open it from the menu bar:

**Tools → Device Configuration**

> Requires the controller to be launched with `GUI:=true`.

---

### Motor Calibration

> [!WARNING]
> **Calibration permanently redefines the firmware's angle origin.**
> All joints will be zeroed to whatever posture the hand is in **at the moment calibration runs**.
> Only use this when the hand is:
> - **Torque off** (no powered joints)
> - **No load** (nothing held, fingers free)
> - In the **intended reference posture**
>
> Once triggered, **there is no undo**. The previous zero reference is lost.

Sends the `calibration` command through `allegroHand/lib_cmd`, identical to pressing **`C`** on the keyboard node.

1. Turn torque off and move the hand to the intended reference posture manually.
2. Open the Device Configuration panel.
3. Click **Calibrate**.
   - The GUI publishes `lib_cmd: "calibration"` → `AllegroNodeGrasp` calls `canDevice->calibrate()`.
   - On **Ethernet (UDP) mode**, this sends `JAC_REQ` (0x25) over UDP to the firmware.
4. A status message confirms the command was sent.

See also: [Motor calibration (joint / encoder zero)](#️-motor-calibration-joint--encoder-zero) for the full explanation of post-calibration angle behavior.

---

## ⚙️ Motor calibration (joint / encoder zero)

Motor calibration adjusts the hand’s **encoder / joint angle reference** per firmware rules. Use only when appropriate: **torque off**, **no load**, clear workspace.

### What you have to do

1. Move the hand to the **reference posture** to treat as the new electrical/mechanical zero baseline (again: safe, no load).
2. Trigger calibration (ROS 2 `lib_cmd`, keyboard **`C`**, or your own tooling that calls the same driver entry points).  
3. The driver sends the calibration request on the **active communication path** (CAN command or V5B2 **`JAC_REQ` (0x25)** over **UDP** on Ethernet mode).



### Trigger from ROS 2

* Publish on **`allegroHand/lib_cmd`** (apply your launch `NUM` remap if needed):

  ```bash
  ros2 topic pub --once /allegroHand_0/lib_cmd std_msgs/msg/String "{data: calibration}"
  ```

* Or use the **keyboard** node and press **`C`** (`allegro_hand_keyboard`).

* Code path: `lib_cmd` string **`calibration`** → `AllegroNodeGrasp::libCmdCallback` → `canDevice->calibrate()` (virtual; implementation follows **CAN** or **Ethernet (UDP)** depending on the `CAN` launch argument).

### ⚠️ Effect on reported angles (read before running)

After calibration **finishes successfully**, the firmware **redefines the angle origin**: **whatever pose the hand was in during calibration is taken as the new reference**, so **reported joint angles will read as 0° for the joints that the firmware maps that way.** In the same convention, **`joint13` is reported as 90°** (not 0°).


---

## 🔗 Useful Links
* **Official Allegro Hand Website:** [https://www.allegrohand.com/](https://www.allegrohand.com/)
* **Community Forum:** [https://www.allegrohand.com/forum](https://www.allegrohand.com/forum)
* **Allegro Hand V5 Plus ROS2 Package:** [Wonik Robotics GitHub](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5)
   

---


