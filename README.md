# Variable Tilt Hexacopter

## Overview

This repository contains a **Simscape/Simulink model** and a **ROS2 Gazebo simulation setup** for a **Variable Tilt Hexacopter**, designed to simulate and analyze the dynamics and control of a hexacopter with adjustable propeller tilting angles. The project supports both offline modeling and real-time robotic simulation to facilitate research in **geometric control, flight dynamics**, and **ROS-based deployment**.



## Repository Structure

```plaintext
📂 VariableTiltHexacopter
├── 📁 ros_ws/                               # ROS2 workspace
│   └── 📁 src/
│       └── 📁 hexacopter_description/
│           ├── 📁 launch/                   # ROS2 launch files
│           ├── 📁 hexacopter_description/   # ROS2 nodes, etc.
│           ├── 📁 urdf/                     # Xacro/URDF description of the hexacopter
│           ├── 📁 worlds/                   # Gazebo SDF environments
│           ├── 📄 package.xml
│           └── 📄 setup.py
│       └── 📁 geometric_controllers/
│           ├── 📁 launch/                   # ROS2 launch files
│           ├── 📁 geometric_controllers/    # Python files and ROS2 nodes for control, etc.
│           ├── 📄 package.xml
│           └── 📄 setup.py
├── .gitignore
└── README.md
```



## Getting Started

### Prerequisites

#### ROS2 Simulation
- **ROS2** (Humble)
- **Gazebo Ignition** (Harmonic)
- **colcon** (ROS2 build tool)
- **xacro**, `ros_ign_gazebo` plugins, and other common ROS2 packages

#### Installing Extra Dependencies for ROS2

Start with running: ` sudo apt-get update `

- **ROS Gazebo**: needed to run gazebo from ROS
   ```
   sudo apt-get install ros-humble-ros-gz
   ```
- **ROS Gazebo Bridge**: Needed to use Gazebo from ROS.  
   ```
   sudo apt-get install ros-humble-ros-ign-bridge
   ```
- **Xacro**:    
   ```
   sudo apt install ros-humble-xacro
   ```




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

### 2. Source the workspace
```bash
src-hexacopter
```

### 3. Basic Demo: Launch the hexacopter in Gazebo Harmonic

- Just Launch the Robot in an Empty world:
   ```bash
   ros2 launch hexacopter_description spawn_robot.launch.py
   ```
   You can specify the start position of the robot:
   ```bash
   ros2 launch hexacopter_description spawn_robot.launch.py start_pos:='5.0 0.5 2.0'
   ```

- Launch the robot with simple fixed open-loop control. angles - tilt angles in rad. motor_speeds - propeller motor speeds in rad/s.
   ```bash
   ros2 launch hexacopter_description simple_control.launch.py angles:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" motor_speeds:="[800.0, 800.0, 800.0, 800.0, 800.0, 800.0]"
   ```

- Launch the robot with simple hover control.  
   - allocation_method - control allocation method (fixed_tilt or variable_tilt).  
   - tilt_angle - tilt angle of the rotors for fixed tilt in radians (one value for all).  
   - hover_altitude -  
   - hover_gain -  
   **Use any of the following commands:** depending on the default values you may want to override.
   ```bash
   ros2 launch hexacopter_description hover_control.launch.py hover_altitude:=5.0 hover_gain:=12.0
   ```
   ```bash
   ros2 launch hexacopter_description hover_control.launch.py allocation_method:="fixed_tilt" tilt_angle:="0.52"
   ```
   ```bash
   ros2 launch hexacopter_description hover_control.launch.py hover_altitude:=5.0 hover_gain:=12.0 allocation_method:="fixed_tilt" tilt_angle:="0.52"
   ```

### 4. Geometric Control Demo
- Launch the robot with simple hover control.  
   - hover_altitude -   
   - Kp_att - potential gain for orientation control (e.g.: "[1.0, 1.0, 1.0]")   
   - Kp_pos - potential gain for position control (e.g.: "[1.0, 1.0, 1.0]")   
   - Kd - derivative gain; angular first, then linear (e.g.: "[1.0, 1.0, 1.0, 5.0, 5.0, 5.0]")   
   - allocation_method - control allocation method (fixed_tilt or variable_tilt).  
   - tilt_angle - tilt angle of the rotors for fixed tilt in radians (one value for all).  
   - world - the environment of the simulation. (empty.sdf or industrial_warehouse.sdf)  
   **Use any of the following commands:** you may add more parameters to override the default.
   ```bash
   ros2 launch geometric_controllers hover_control.launch.py hover_altitude:=5.0
   ```

- Launch the robot with trajectory tracking. 
   - path - path type: circle, square, infinity, takeoff_land, hover
   - path_scale - path scaling factor
   - path_period - time to complete one cycle of the path
   - path_altitude - altitude of the generated path
   - Kp_att - potential gain for orientation control (e.g.: "[1.0, 1.0, 1.0]")   
   - Kp_pos - potential gain for position control (e.g.: "[1.0, 1.0, 1.0]")   
   - Kd - derivative gain; angular first, then linear (e.g.: "[1.0, 1.0, 1.0, 5.0, 5.0, 5.0]")   
   - allocation_method - control allocation method (fixed_tilt or variable_tilt).  
   - tilt_angle - tilt angle of the rotors for fixed tilt in radians (one value for all).  
   - world - the environment of the simulation. (empty.sdf or industrial_warehouse.sdf)  
   **Use the following commands:** you may add more parameters to override the default.
   ```bash
   ros2 launch geometric_controllers path_following.launch.py path:=square
   ```

### 5. Adaptive Geometric Control Demo
- Same as the geometric control commands except that the launch name file is changed.  
   You can change the mass of the attached object in `attached_mass.sdf` file under the `hexacopter_description/urdf` folder.
   ```
   ros2 launch geometric_controllers path_following_adaptive.launch.py path:=square
   ```

### Running entire pipeline (Using Single Script) 

__Warning__: Not advised. It does not stop the Gazebo Simulation correctly.  

The pipeline involves: running simulation, recording ROS bags and plotting the results. The entire pipeline is implemented in the `bash_scripts` folder. To run the pipeline, simple change the `controller_type` and the `path` to the desired values in the `bash_scripts/run_all.sh` file. Then, run the code:

```
cd ~/VariableTiltHexacopter/bash_scripts   
bash run_all.sh
```

### Running entire pipeline (In Separate Terminals)  

The pipeline involves: running simulation, recording ROS bags and plotting the results. 
- Launch Simulation
   ```
   src-hexacopter
   ros2 launch geometric_controllers path_following.launch.py path:=hover path_period:=60.0
   ```

- Launch ROS Bag
   ```
   cd ~/VariableTiltHexacopter/ros_bags/bags
   ros2 bag record \
      /model/variable_tilt_hexacopter/odometry \
      /model/variable_tilt_hexacopter/desired_wrench \
      /model/variable_tilt_hexacopter/plot/motor_speed \
      /model/variable_tilt_hexacopter/plot/tilt_angle \
      /model/variable_tilt_hexacopter/desired_pose \
      /model/variable_tilt_hexacopter/desired_velocity \
      -o geometric_control_PD_square_bag
   ```

- Start Simulation
   Press the `space bar` to quickly start the simulation. First, stop the ROS Bag by using `Ctrl + C`. Then, press `Ctrl + C` in the terminal where you launched Gazebo to simply stop the Simulation.

- Plot Result

   **Note**: The name of the bag (in front of the tag `-b`) must be same as the name passed as the output of the ROS Bag (in front of tag `-o`).

   ```
   cd ~/VariableTiltHexacopter/ros_bags
   python3 plot_hexacopter.py \
      -b bags/geometric_control_PD_square_bag \
      -o plot/PD_square \
      -d 80 \
      -s 2
   ```

## Important Development Notes

### Creating SDF file from URDF Xacro.
In developing the robots, we start by defining the ```variable_tilt_hexacopter.urdf.xacro``` file. This file is converted to URDF using:
```
ros2 run xacro xacro -o ../path/variable_tilt_hexacopter.urdf ../path/variable_tilt_hexacopter.urdf.xacro
```

Then, this URDF file is converted to SDF file using:
```
gz sdf -p ../path/variable_tilt_hexacopter.urdf > ../path/variable_tilt_hexacopter.sdf
```

> Note: You need to replace ```../path/``` with the actual path. However, if you are in the URDF folder, you can simply omit it.  
> *You need to **manually copy the plugins** to the SDF file from the URDF file.* It is not created automatically.



## Features & Roadmap

✅ URDF/Xacro model of the hexacopter  
✅ ROS2 integration with Gazebo Harmonic  
✅ Simple world environment: `empty.sdf`  
✅ ROS2 nodes for tilt control and motor commands  
⬜ GUI for Control  
✅ Control Allocation (Fixed Tilt)  
⬜ Control Allocation (Variable Tilt)  
✅ Control Allocation (Optimization-based)  
✅ Open Loop: Hover Control  
✅ Geometric-based Control: PD Control   
⬜ Geometric-based Control: Feedback Linearized Control   
⬜ Geometric-based Control: Adaptive Control   
✅ Realistic world environments  
✅ Trajectory Planning  
✅ Integrate sensor plugins (IMU, camera)  
⬜ State Estimation and Mapping  
⬜ Extend to Floating-Base Manipulators  
⬜ ... New Improvements  



## Contributing

Feel free to fork this repo and contribute via pull requests. Open issues for bugs, feature suggestions, or integration help.



## Troubleshooting

- One common error when bypassing the default parameters when launching from terminal is type error. Although the error message usually doesn't make this apparent. Ensure that parameters are given correct type. For a double type for example, `0` will not be accepted, rather, it should be `0.0`.



## License

[MIT License](LICENSE) – Free to use, modify, and distribute.

