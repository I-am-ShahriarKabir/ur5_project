# UR5 Project: ROS Simulation, Data Collection, and ML/AI Analysis

This project demonstrates an end-to-end solution for simulating a UR5 robot in Gazebo, collecting robot data (joint states and camera images), and performing ML/AI analysis on the stored data. The project is organized as a ROS package for the simulation and data collection components along with a separate folder for ML experiments.

---

## Project Directory Structure

Below is a diagram of the project's folder structure:

```bash
ur5_project/
├── README_1.md                # Documentation for Task 1 (Simulation)
├── README_2.md                # Documentation for Task 2 (Data Collection)
├── README_3.md                # Documentation for Task 3 (ML/AI)
├── README_4_git_dir.md        # Documentation for Task 4 (Pushing to Git)
├── assigment_per_DataEngineer.pdf
├── .gitignore                 # Specifies files/folders to ignore in Git
├── catkin_ws/                 # Catkin workspace containing ROS packages
│   ├── build/                 # Build artifacts (ignored)
│   ├── devel/                 # Development space (ignored)
│   └── src/
│       ├── CMakeLists.txt     # Top-level CMake file for the workspace
│       ├── universal_robot/   # Cloned ROS-Industrial universal_robot repository
│       └── ur5_controller/    # Custom ROS package for the UR5 project
│           ├── CMakeLists.txt # CMake file for the ROS package
│           ├── package.xml    # Package manifest
│           ├── launch/        # Launch files
│           │   └── ur5_simulation.launch  # Launches simulation, control, and data collection
│           └── src/           # Source code for the ROS package
│               ├── ur5_sine_wave_commander.py  # Commands the UR5 with sine wave trajectories
│               ├── ur5_sine_wave_publisher.py    # (Alternate) Publishes joint states as sine waves
│               ├── data_collector.py             # Collects joint and camera data, writes CSV
│               └── joint_data.csv                # CSV file generated during runtime
└── ml_experiments/            # Folder for ML/AI experiments
    └── ml_example.py          # Python script/notebook for data analysis and prediction
```

---

## Steps Taken to Push

1. **Organize Project Files:**  
   - Created a ROS package (`ur5_controller`) inside `catkin_ws/src/` with the following:
     - **src/**: Contains Python scripts (UR5 control, data collection).
     - **launch/**: Contains launch files (e.g., `ur5_simulation.launch`).
     - **CMakeLists.txt** and **package.xml**: Required for building and running the ROS package.
   - Created a separate folder (`ml_experiments/`) for ML experiments, containing the `ml_example.py` file.
   - Placed all README documents (for Tasks 1–4) in the main folder (`Neura_task/`).

2. **Create a .gitignore File:**  
   A `.gitignore` file was added at the top-level (`Neura_task/`) to ignore build directories, temporary files, and unwanted files such as:
   - `catkin_ws/build/` and `catkin_ws/devel/`
   - Python cache files (`__pycache__`, `*.pyc`)
   - Jupyter Notebook checkpoints
   - ROS log files, system files (e.g., `.DS_Store`), and the specific `assigment_per_DataEngineer.pdf:Zone.Identifier` file.

3. **Initialize a Git Repository:**  
   From the main folder (`Neura_task/`), the following commands were executed:
   ```bash
   cd ~/Neura_task
   git init
   git add .
   git commit -m "Initial commit: UR5 simulation, data collection, and ML/AI pipeline"

3. **Create and Push to Remote Repository:**  
   - A remote repository on GitHub was created.
   - The remote was added, and the code was pushed:
        ```bash
        git remote add origin https://https://github.com/I-am-ShahriarKabir/ur5_project
        git branch -M main
        git push -u origin main