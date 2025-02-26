# Data Collection Pipeline for Simulated UR5 Robot

This task implements a data collection pipeline for the simulated UR5 robot running in Gazebo. The pipeline collects robot data (joint angles/configurations and camera images) and stores it locally on an Edge Gateway. Optionally, this data can be synced with the cloud using additional modules.

---

## 1. Software Architecture

The overall system is structured as follows:

     +-------------------------+
     |   UR5 Simulation in     |  
     |        Gazebo           |
     |                         |
     |  Publishes Topics:      |
     |   - /joint_states       |  
     |   - /camera/image_raw   |
     +------------+------------+
                  |
                  v
     +-------------------------+
     |   Data Collector Node   |
     |   (data_collector.py)   |
     |                         |
     |  Subscribes to:         |
     |   - /joint_states       | --> Stores data in CSV (joint_data.csv)
     |   - /camera/image_raw   | --> Displays real-time camera view (via OpenCV)
     +------------+------------+
                  |
                  v
     +-------------------------+
     |   Optional Cloud Sync   |
     |   (e.g., MQTT/HTTP Agent)|
     +-------------------------+


**Explanation:**

- **UR5 Simulation in Gazebo:**  
  The simulation (using the universal_robot package) publishes the robot's joint states and camera images on the appropriate topics.

- **Data Collector Node:**  
  The `data_collector.py` node subscribes to `/joint_states` and `/camera/image_raw` (if available) and:
  - Writes joint state data (joint angles) to a CSV file (`joint_data.csv`).
  - Uses OpenCV to display the camera feed in real time.

- **Optional Cloud Sync:**  
  Although not implemented in this version, you could add a service or agent that forwards the locally stored data (CSV and images) to a cloud platform using protocols like MQTT or HTTP.

- **Real-time Visualization with rqt_plot:**  
  Additionally, **rqt_plot** is launched to visualize the first joint’s position in real time.

---

## 2. Files and Their Explanations

### a. data_collector.py

- **Location:** `ur5_controller/src/data_collector.py`
- **Purpose:**  
  - Subscribes to the `/joint_states` topic to collect joint configuration data.
  - Optionally subscribes to the `/camera/image_raw` topic to display real-time camera images.
  - Writes the joint state data to a CSV file (`joint_data.csv`) for offline analysis.
- **Key Functions:**
  - **`joint_callback(msg)`**: Extracts timestamp and joint positions from the `JointState` message and writes them as a new row in the CSV.
  - **`image_callback(msg)`**: Converts ROS image messages to OpenCV format and displays them in a window.
  - **`shutdown()`**: Closes the CSV file and OpenCV windows gracefully on node shutdown.

### b. ur5_sine_wave_commander.py

- **Location:** `ur5_controller/src/ur5_sine_wave_commander.py`
- **Purpose:**  
  - Publishes `JointTrajectory` commands to the `/eff_joint_traj_controller/command` topic.
  - Generates sine wave joint positions for each joint and sends them as trajectory commands so that the UR5 robot moves accordingly in the simulation.

### c. Launch File (ur5_simulation.launch)

- **Location:** `ur5_controller/launch/ur5_simulation.launch`
- **Purpose:**  
  - Combines all nodes into a single launch file.
  - Launches the Gazebo simulation using the universal_robot package.
  - Starts the sine-wave commander node, the data collector node, and **rqt_plot** for real-time plotting.
- **Contents:**
  ```xml
  <launch>
    <!-- Launch the Gazebo simulation (UR5 simulation from the universal_robot package) -->
    <include file="$(find ur_gazebo)/launch/ur5_bringup.launch" />
    
    <!-- Launch the sine-wave commander node (this commands the robot) -->
    <node pkg="ur5_controller" type="ur5_sine_wave_commander.py" name="ur5_sine_wave_commander" output="screen" />
    
    <!-- Launch the data collection node (this collects joint states and camera images) -->
    <node pkg="ur5_controller" type="data_collector.py" name="data_collector" output="screen" cwd="node" />
    
    <!-- Launch rqt_plot to visualize joint state positions, plotting the first joint's position -->
    <node pkg="rqt_plot" type="rqt_plot" name="rqt_plot" args="/joint_states/position[0]" output="screen" />
  </launch>


## 3. Install, Build, and Run

1. **Install Required Packages:**
    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-cv-bridge ros-noetic-rqt-plot

2. **Build the Custom Package:**
    ```bash
    cd ~/ur5_project/catkin_ws
    catkin_make
    source devel/setup.bash

3. **Running the Pipeline:**
    - Open a terminal, source your workspace, and start roscore:
        ```bash
        roscore
    
    -  Launch the Data Collection Pipeline:
        In a new terminal (with the workspace sourced), run:
        ```bash
        roslaunch ur5_controller ur5_simulation.launch

    - What to Expect:
        **Gazebo Simulation:**
            The UR5 robot is spawned and controlled via the sine-wave commands.

        **Data Collector Node:**
           a. Joint states are being recorded in a file named joint_data.csv (created in the node’s working directory, typically in the package’s src  folder since cwd="node" is used).
           b. If a camera is available and publishing on /camera/image_raw, a window titled "Camera View" will open displaying the feed.
           c. Real-time Visualization:
                rqt_plot is launched and plots the first element of the /joint_states/position array (i.e., the position of the first joint) in real time.
           d. CSV File:
                You can open joint_data.csv to inspect the logged joint states.


