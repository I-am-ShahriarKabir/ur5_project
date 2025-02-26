# UR5 Robot Simulation, Data Collection, and ML/AI Pipeline

This project demonstrates an end-to-end solution for simulating a UR5 robot in Gazebo using ROS, collecting data from the simulation (joint angles and camera images), and expanding the pipeline with ML/AI capabilities to analyze and predict joint behavior.

## Overview

- **Task 1: UR5 Simulation and Control**  
  - Sets up a simulation environment using the open-source [universal_robot](https://github.com/ros-industrial/universal_robot) package.
  - Implements a custom ROS node that publishes desired joint angles as sine waves.
  - Includes a launch file to start the simulation and control nodes.  
  _See [README_1.md](README_1.md) for details._

- **Task 2: Data Collection Pipeline**  
  - Implements a data collector node that subscribes to joint state and camera topics, storing joint data in a CSV file.
  - Presents the software architecture and real-time data visualization (including rqt_plot).  
  _See [README_2.md](README_2.md) for details._

- **Task 3: ML/AI Expansion**  
  - Loads the stored CSV data, prepares the dataset, and trains ML models (Polynomial Regression and an MLP Regressor) to predict joint angles.
  - Evaluates model performance using metrics such as MSE, MAE, and R², and visualizes actual vs. predicted data.
  _See [README_3.md](README_3.md) for details._

- **Task 4: Project Organization & Git Setup**  
  - Organizes the project into a well-structured folder with `src`, `include`, `launch`, and configuration files (CMakeLists.txt, package.xml).
  - The project is pushed to GitHub as a complete repository.  
  _See [README_4.md](README_4_git_dir.md) for details._

## Project Directory Structure

```bash
Neura_task/
├── README_1.md                # Task 1: UR5 Simulation and Control
├── README_2.md                # Task 2: Data Collection Pipeline
├── README_3.md                # Task 3: ML/AI Expansion
├── README_4_git_dir.md        # Task 4: Project Organization & Git Setup
├── assigment_per_DataEngineer.pdf
├── .gitignore                 # Specifies files/folders to ignore in Git
├── catkin_ws/                 # Catkin workspace containing ROS packages
│   ├── build/                 # Build artifacts (ignored)
│   ├── devel/                 # Development space (ignored)
│   └── src/
│       ├── CMakeLists.txt     # Top-level CMake file for the workspace
│       ├── universal_robot/   # ROS-Industrial universal_robot submodule
│       └── ur5_controller/    # Custom ROS package for the UR5 project
│           ├── CMakeLists.txt # CMake file for the ROS package
│           ├── package.xml    # Package manifest
│           ├── launch/        # Launch files
│           │   └── ur5_simulation.launch  # Launches simulation, control, and data collection
│           └── src/           # Source code for the ROS package
│               ├── ur5_sine_wave_commander.py  # Commands UR5 using sine wave trajectories
│               ├── ur5_sine_wave_publisher.py    # (Alternate) Publishes joint states as sine waves
│               ├── data_collector.py             # Collects joint and camera data, writes CSV
│               └── joint_data.csv                # CSV file generated during runtime
└── ml_experiments/            # Folder for ML/AI experiments
    └── ml_example.py          # Python script for data analysis and prediction
