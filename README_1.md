# UR5 Simulation with Sine Wave Joint Control

This task demonstrates a simulation environment for a UR5 robot in Gazebo using ROS (Noetic) and the [universal_robot](https://github.com/ros-industrial/universal_robot) package. In this task, we control the UR5 robot by publishing joint commands in the form of sine waves.

## Overview

The task consists of the following components:
- **Gazebo Simulation:** Uses the `ur_gazebo` package from the universal_robot repository to simulate the UR5 robot.
- **Custom ROS Nodes:**
  - **ur5_sine_wave_publisher.py:** Publishes `sensor_msgs/JointState` messages with sine wave positions on the `/joint_states` topic. This node is useful for demonstrating joint state publishing (typically used for feedback).
  - **ur5_sine_wave_commander.py:** Publishes `trajectory_msgs/JointTrajectory` messages to the `/eff_joint_traj_controller/command` topic. This is the proper command interface for the robot controllers and causes the UR5 robot to move in simulation.
- **Launch File:** A launch file that starts the Gazebo simulation (including the UR5 model) and launches the custom sine-wave commander node.

## Prerequisites and Installation

1. **ROS Noetic and Gazebo:**  
   Ensure you have [ROS Noetic](http://wiki.ros.org/noetic) and Gazebo installed on your system.

2. **Universal Robot Packages:**  
   Clone the universal_robot repository into your Catkin workspace and build it:
   ```bash
   cd ~/Neura_task/catkin_ws/src
   git clone https://github.com/ros-industrial/universal_robot.git
   cd ~/Neura_task/catkin_ws
   catkin_make
   source devel/setup.bash

3. **ROS Controllers:**  
   Install the necessary ros_control packages:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control

4. **Custom Package Setup:**  
   The custom package ur5_controller was created with:
   ```bash
   cd ~/Neura_task/catkin_ws/src
   catkin_create_pkg ur5_controller rospy std_msgs sensor_msgs trajectory_msgs

Inside ur5_controller, the following directory structure is used:
ur5_controller/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── ur5_simulation.launch
└── src/
    ├── ur5_sine_wave_publisher.py
    └── ur5_sine_wave_commander.py

## File Descriptions

1. **ur5_sine_wave_publisher.py**
- Purpose:
    Publishes JointState messages with sine wave positions on the /joint_states topic. This node is intended to demonstrate joint state publishing. However, most controllers use command topics for actual movement.

- Key Points:

    - Uses Python3 and publishes at 10 Hz.
    - Computes sine wave positions for all UR5 joints:
        shoulder_pan_joint
        shoulder_lift_joint
        elbow_joint
        wrist_1_joint
        wrist_2_joint
        wrist_3_joint

2. **ur5_sine_wave_commander.py**
- Purpose:
    Publishes JointTrajectory messages to command the robot via the /eff_joint_traj_controller/command topic. This is the appropriate method to drive the robot joints in the simulation using trajectory commands.

- Key Points:

    - Also publishes at 10 Hz.
    - Computes sine wave positions for each joint and packages them into a JointTrajectoryPoint with a duration of 1 second to reach the target.
    - This node is used to directly command the robot’s movement in the Gazebo simulation.

3. **Launch File (ur5_simulation.launch)**
- Purpose:
    Starts both the Gazebo simulation and the custom sine-wave commander node.

- Usage:
    To launch the simulation and node, run:
    ```bash
    roslaunch ur5_controller ur5_simulation.launch

## Building and Running

1. **Build the Workspace:**
    ```bash
    cd ~/Neura_task/catkin_ws
    catkin_make
    source devel/setup.bash

2. **Start ROS Master (roscore):**
    In a separate terminal (make sure to source your workspace there as well), start the ROS master:
    ```bash
    roscore

3. **Launch the Simulation:**
    Once the ROS master is running, launch the simulation:
    ```bash
    roslaunch ur5_controller ur5_simulation.launch

4. **Testing:**
    To observe the published messages:
    ```bash
    rostopic echo /joint_states
    rostopic echo /eff_joint_traj_controller/command
