# Variable Tilt Hexacopter

## Overview

This repository contains a **Simscape/Simulink model** and a **ROS2 Gazebo simulation setup** for a **Variable Tilt Hexacopter**, designed to simulate and analyze the dynamics and control of a hexacopter with adjustable propeller tilting angles. The project supports both offline modeling and real-time robotic simulation to facilitate research in **geometric control, flight dynamics**, and **ROS-based deployment**.

---

## Repository Structure

```plaintext
📂 VariableTiltHexacopter
├── 📁 matlab
    ├── 📁 modelling_scripts/             # MATLAB scripts for modeling and analysis
│   │   ├── 📄 compute_params.slx         # Simulink for evaluating inertia tensors
│   │   └── 📄 fixed_wrench_analysis.mlx  # Livescript for wrench evaluation
│   ├── 📄 param.m                        # MATLAB script for hexacopter parameters
│   └── 📄 simulation.slx                 # Simulink model
├── 📁 ros_ws/                            # ROS2 workspace
│   └── 📁 src/
│       └── 📁 hexacopter_description/
│           ├── 📁 launch/                # ROS2 launch files
│           ├── 📁 resource/              # Package resources
│           ├── 📁 scripts/               # (Planned) ROS2 nodes for control, etc.
│           ├── 📁 urdf/                  # Xacro/URDF description of the hexacopter
│           ├── 📁 worlds/                # Gazebo SDF environments
│           ├── 📄 package.xml
│           └── 📄 setup.py
├── .gitignore
└── README.md
```

---

## Getting Started

### Prerequisites

#### MATLAB Simulation
- **MATLAB & Simulink** (R2021b or later recommended)
- **Simscape Multibody**

#### ROS2 Simulation
- **ROS2** (e.g., Foxy, Galactic, or Humble)
- **Gazebo Fortress/Ignition**
- **colcon** (ROS2 build tool)
- **xacro**, `ros_ign_gazebo` plugins, and other common ROS2 packages

#### Installing Extra Dependencies for ROS2

Start with running: ` sudo apt-get update `

- **ROS Gazebo Bridge**: Needed to use Gazebo from ROS.  
   ```
   sudo apt-get install ros-humble-ros-ign-bridge
   ```
- **Xacro**:    
   ```
   sudo apt install ros-humble-xacro
   ```


---

## MATLAB Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/olanrewajufarooq/VariableTiltHexacopter.git
   cd VariableTiltHexacopter
   ```

2. Open MATLAB and navigate to the repo folder.

3. Run the parameter setup:
   ```matlab
   run('modelling_scripts/param.m')
   ```

4. Open and simulate the model:
   ```matlab
   open('modelling_scripts/simulation.slx')
   ```

   > View the physical model in Mechanics Explorer.

---

## ROS2 + Gazebo Usage

### 0. Setup the Bash for workspace building and sourcing
- Open Bashrc
   ```cmd
   gedit ~/.bashrc
   ```
- Add these command to automatically implement all commands for building
   ```cmd
   build-hexacopter() {
      cd ~/VariableTiltHexacopter/ros_ws
      rosdep install -i --from-path src --rosdistro humble -y
      rm -rf build install log
      source /opt/ros/humble/setup.bash
      colcon build
      source install/local_setup.bash
   }
   ```

- Add these command to automatically implement all commands for sourcing
   ```cmd
   src-hexacopter() {
      cd ~/VariableTiltHexacopter/ros_ws
      source /opt/ros/humble/setup.bash
      source install/local_setup.bash
   }
   ```
- Save and close the bashrc. Then, source the bashrc:
   ```cmd
   source ~/.bashrc
   ```

 > With these, you will not need to repeatedly type these boring commands for building or sourcing your ros workspace. However, be careful that the `cd ~/VariableTiltHexacopter/ros_ws` is opening the right directory, otherwise, use the correct directory path.

### 1. Build the workspace
```bash
build-hexacopter
```

### 2. Launch the hexacopter in Gazebo Fortress
```bash
ros2 launch hexacopter_description spawn_robot.launch.py
```

---

## Features & Roadmap

✅ URDF/Xacro model of the hexacopter  
✅ ROS2 launch integration with Gazebo Fortress  
✅ World environments: `empty.sdf`, `warehouse.sdf`  

🚧 In Development / Planned:
- ROS2 nodes for tilt control and flight dynamics
- Sensor plugins (IMU, camera, lidar)
- Teleop via joystick/keyboard
- RViz integration and visualization enhancements

---

## Contributing

Feel free to fork this repo and contribute via pull requests. Open issues for bugs, feature suggestions, or integration help.

---

## License

[MIT License](LICENSE) – Free to use, modify, and distribute.

