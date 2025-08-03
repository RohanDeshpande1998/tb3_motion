# 🐢 TurtleBot3 Motion Controller

A modular ROS 2-based project for generating and executing smooth trajectories for TurtleBot3 using B-splines and Pure Pursuit control. It supports trajectory visualization in RViz and error analysis with live plots.

---

## 📁 Project Structure

tb3_motion_ws/  
├── src/  
│   └── tb3_controller/  
│       ├── pure_pursuit_node.py        - Follows a generated path using Pure Pursuit with PID  
│       ├── trajectory_generator.py     - B-spline generation from waypoints and initial pose  
│       ├── plotter_node.py             - Plots tracking and orientation errors, publishes paths  
│       ├── launch/  
│       │   ├── tb3_simulation.launch.py  - Complete launch for sim + control + plot  
│       │   └── tb3_rviz.launch.py        - Launch RViz with planned and actual path  
│       ├── config/  
│       │   └── waypoints.yaml           - List of waypoints for the trajectory  
│       └── rviz/  
│           └── model.rviz               - RViz config for visualization  
└── data/  
    └── trajectory.npz                  - Auto-generated trajectory  

---

## 🚀 Features

- B-spline trajectory generation with trapezoidal velocity profile  
- Waypoint support via YAML config  
- Pure Pursuit controller using proportional gains  
- Real-time error plots and RViz visualization  
- Designed for Gazebo simulation, tested on TurtleBot3 Burger  

---

## ⚙️ Dependencies

- ROS 2 (tested on Humble)  
- Python libraries: `numpy`, `matplotlib`, `scipy`  
- `tf_transformations` for quaternion math  

Install Python dependencies:  
pip install numpy matplotlib scipy

---

## 🛠️ Build Instructions

cd ~/tb3_motion_ws  
colcon build --packages-select tb3_controller  
source install/setup.bash  

---

## ▶️ Run the Simulation

Launch the full stack:

ros2 launch tb3_controller tb3_simulation.launch.py x:=0.0 y:=0.0 yaw:=0.0

RViz (optional):

ros2 launch tb3_controller tb3_rviz.launch.py

---

## 📈 Visual Output

- RViz Topics: `/actual_path`, `/planned_path`  
- Live Plots: tracking & orientation error using matplotlib

---

## 📝 Waypoints Config

Edit config/waypoints.yaml to set custom waypoints:

waypoints:
  - [1.0, 0.0]
  - [1.5, 1.0]
  - [2.0, 0.5]

---

## 🧠 Control Logic

Trajectory Generator:
- Adds a ghost point behind initial pose to guide the initial curvature of the robot about its heading
- Fits B-spline through waypoints 
- Applies trapezoidal velocity profile

Controller (Pure Pursuit PID):
- Follows trajectory using v = K_RHO * rho, w = K_ALPHA * alpha  
- Publishes /cmd_vel and error topics

Plotter Node:
- Subscribes to errors and odometry  
- Plots and publishes /actual_path, /planned_path  

---

## 🧠 Design Choices, Algorithms & Architecture

### Architecture Overview

The system is composed of three modular ROS 2 nodes:
- Trajectory Generator
- Pure Pursuit Controller
- Plotter Node

Each component is isolated promoting modularity and ease of debugging.

### Algorithms & Design Decisions

| Component            | Design Choice                           | Reason |
|---------------------|------------------------------------------|--------|
| Trajectory          | B-spline with ghost point and waypoints  | Smooth, continuous path generation |
| Timing              | Trapezoidal velocity profile             | Controlled, realistic speed profile |
| Controller          | Pure Pursuit with proportional control   | Simple, reliable for mobile robots |
| Communication       | ROS 2 pub/sub                            | Scalable, standard in robotics |
| Visualization       | RViz + matplotlib                        | Spatial and error-time domain views |
| Configuration       | YAML + CLI args                          | Human-readable and flexible |

### Control Logic

- Pure Pursuit:
  - rho = distance to lookahead point
  - alpha = heading error
  - v = K_RHO * rho
  - w = K_ALPHA * alpha

- Trajectory Generator:
  - Adds ghost point behind robot
  - B-spline interpolation
  - Trapezoidal timing

- Plotter:
  - Subscribes to odometry and error topics
  - Publishes actual and planned paths
  - Live matplotlib updates

---

## 🤖 Extending to a Real Robot

### Replace Simulation Components

| Simulated Element | Real Robot Equivalent |
|-------------------|------------------------|
| Gazebo            | Physical world         |
| /odom from Gazebo | /odom from encoders/IMU |
| /cmd_vel to sim   | /cmd_vel to base controller |
| spawn_entity.py   | Turn on robot manually |

### Launch on Real Robot

1. Bring up the robot:

export TURTLEBOT3_MODEL=burger  
ros2 launch turtlebot3_bringup robot.launch.py

2. Run control nodes:

ros2 run tb3_controller trajectory_generator x y yaw waypoint.yaml  
ros2 run tb3_controller pure_pursuit_node  
ros2 run tb3_controller plotter_node

### Additional Considerations

- Tune gains for real-world behavior
- Keep speeds low for stability
- Validate topics (/odom, /cmd_vel) and remap as needed
- Use rqt_plot/RViz remotely

### Future Extensions

- Add obstacle avoidance using /scan
- Use AMCL or SLAM for localization
- Upgrade controller (e.g., MPC)
- Build interactive waypoint UI

---

## 📜 License

Apache License 2.0

---

## 👤 Author

Rohan Deshpande  
TB3 Motion Planner | Simulation & Control | ROS2
