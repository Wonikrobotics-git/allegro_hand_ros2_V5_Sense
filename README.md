# allegro_hand_ros2_V5_Sense

This is the official release to control Allegro Hand V5 Sense with ROS2(**Only V5 Sense supported**). Mostly, it is based on the release of Allegro Hand V5 ROS2 package.
You can find our latest Allegro Hand V5 ROS2 package :(https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5)
And the interfaces and controllers have been improved and rewritten by Soohoon Yang(Hibo) from Wonik Robotics.

We have integrated the core elements of two ROS packages(allegro_hand_description, allegro_hand_parameters) into a main package(allegro_hand_controllers).

**For now, we support three additional action.**
- Visualize Allegro Hand V5 changing in real time using Rviz2.
- Simply control hand with **GUI** tool instead of using keyboard.
- Support **Gain tuning** systems.
- Save **Custom Joint Position** and Load. 

These packages are tested on ROS2 Humble(Ubuntu 22.04). It will likely not work with newer or older versions.

## Useful Links
- Official Allegro Hand Website : https://www.allegrohand.com/
- Community Forum :  https://www.allegrohand.com/forum

## Summary
- Explain Packages.
- Topic description.
- Run main controller nodes.
- How to Use Hand easily.
- Control more than one hand.

## Packages

**From Allegro Hand V5 Sense, the hand is fully based on torque controller.** 

- allegro_hand_controllers : Contain two main nodes for control the hand and urdf descriptions,meshes.
	- node : Receive encoder data and compute torque using `computeDesiredTorque`.
	- grasp : Apply various pre-defined grasps or customized poses.
- allegro_hand_driver : Main driver for sending and receiving data with the Allegro Hand.
- allegro_hand_keyboard : Node that sends the command to control Allegro Hand. All commands need to be pre-defined.
- allegro_hand_gui : Node that control the allegro hand with gui program.
- bhand : Library files for the predefined grasps and actions, available on 64 bit versions.
  - ⚠ **Default: x86-64-bit.** If using a **ARM64**, update the symlink accordingly from [here](https://github.com/Wonikrobotics-git/Bhandlib_ARM). (Will be Update Soon)

## Topic description

- Control
	- /allegroHand_(NUM)/lib_cmd :  Hand command.
 	- /allegroHand_(NUM)/joint_cmd : Desired hand joint positions and control REAL Allegro Hand.
  - /allegroHand_(NUM)/force_chg : Change grasp force of grasping algorithm(grasp_3, grasp_4).
  - /allegroHand_(NUM)/time_chg : Change time of moving target positions of each joints.
  - /allegroHand_(NUM)/envelop_torque : Change torque of envelop command.
- Joint States
  - /allegroHand_(NUM)/joint_states : REAL Allegro Hand current joint positions.
    - Joint Numbering Order

The `Joint Num` sequence consists of 16 joints, ordered as follows:

| Index Range | Finger | Sequence |
| :--- | :--- | :--- |
| `0 ~ 3` | **Index Finger** | Joint 1(base), Joint 2, Joint 3, Joint 4(tip) |
| `4 ~ 7` | **Middle Finger** | Joint 1(base), Joint 2, Joint 3, Joint 4(tip) |
| `8 ~ 11` | **Pinky Finger** | Joint 1(base), Joint 2, Joint 3, Joint 4(tip) |
| `12 ~ 15` | **Thumb** | Joint 1(base), Joint 2, Joint 3, Joint 4(tip) |
- Sensors
  - /allegroHand_(NUM)/tactile_sensors : REAL Allegro Hand current tactile sensors datas.
    - Joint Numbering Order
The `Sensor` sequence consists of 16 sensors(including Palm sensor), ordered as follows:

| Index Range | Finger | Sequence |
| :--- | :--- | :--- |
| `0 ~ 3` | **Thumb** | Palm Sensor, Thumb madi Sensor 1, Thumb madi Sensor 2, Thumb Finger tip Sensor|
| `4 ~ 7` | **Index Finger** | Index Finger madi Sensor 1, Index Finger madi Sensor 2, Index Finger madi Sensor 3, , Index Fingertip Sensor  |
| `8 ~ 11` | **Middle Finger** | Middle Finger madi Sensor 1, Middle Finger madi Sensor 2, Middle Finger madi Sensor 3, , Middle Fingertip Sensor |
| `12 ~ 15` | **Pinky Finger** | Pinky Finger madi Sensor 1, Pinky Finger madi Sensor 2, Pinky Finger madi Sensor 3, , Pinky Fingertip Sensor |   

**With ROS2, you don't need to install PCAN driver anymore!**

If you have already installed PCAN driver, please follow instructions below.
- [Optional] Disable previously installed PEAK-CAN driver.
~~~bash
# Temporarily
sudo rmmod pcan
sudo modprobe peak_usb

# Permanent
sudo rm /etc/modprobe.d/pcan.conf
sudo rm /etc/modprobe.d/blacklist-peak.conf
~~~

## Run main controller nodes

1. Make new workspace.
~~~bash
mkdir allegro_ws
~~~

2. Install necessart package.
~~~bash
sudo apt-get update
sudo apt-get install ros-<distro>-xacro
sudo apt install ros-<distro>-moveit
sudo apt install ros-<distro>-controller-manager
sudo apt install ros-<distro>-joint-state-broadcaster
sudo apt install ros-<distro>-joint-trajectory-controller
~~~

3. Download ROS2 package for Allegro Hand V5 using below command.
~~~bash
cd ~/allegro_ws
git clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_V5_Sense.git
~~~

4. Build.
~~~bash
cd ~/allegro_ws
source /opt/ros/<distro>/setup.bash
colcon build
~~~

5. Launch allegro main node.
~~~bash
source install/setup.bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right
~~~
**You need to write your password to open CAN port**

**Please check 'Launch file instructions below'.**

6. Run allegro hand keyboard node.
~~~bash
cd ~/allegro_ws
source install/setup.bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard
~~~

7. Control Hand using Keyboard command.

## Launch file instructions

Same as the ROS1 package, you can simply control Allegro Hand V5 by launching *allegro_hand.launch.py* . At a minimum, you must specify the handedness and the hand type:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right|left
~~~

Optional arguments:
~~~
VISUALIZE:=true|false (default is false)
GUI:=true|false (default is false)
~~~

- If you want to visualize Allegro Hand on Rviz2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right VISUALIZE:=true
~~~

- If you want to control Allegro Hand with GUI:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right GUI:=true
~~~

## Control more than one hand

To control more than one hand, you must specify CAN port number and Allegro Hand number.

Terminal 1:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right CAN_DEVICE:=can0 NUM:=0
~~~

Termianl 2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=left CAN_DEVICE:=can1 NUM:=1
~~~

To control first hand,

Terminal 3:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_0/lib_cmd
~~~

To control second hand,

Terminal 4:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_1/lib_cmd
~~~

**These are example commands.You may need to change CAN_DEVICE, PORT and NUM arguments according to your system.**

## GUI

These newly added feature function identically to their ROS1 counterparts. Please refer to the ROS1 manual for guidance.
Our latest Allegro Hand V5 ROS1 package : [ROS1](https://github.com/Wonikrobotics-git/allegro_hand_ros_v5)

