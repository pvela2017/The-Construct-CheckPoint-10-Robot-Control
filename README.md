# CheckPoint 10 Robot Control

<a name="readme-top"></a>

## About The Project
In this project the CyberLab Rosbot XL, a real hardware holonomic or omni-directional robot is used. The primary objective is to program the Rosbot XL to navigate through the CyberLab maze successfully. To achieve this, waypoints to guide the robot towards the goal position were provided. Additionally, control algorithms (PID) were tuned to facilitate fast and precise movements, enabling the robot to traverse each waypoint with optimal speed and accuracy. 

![This is an image](images/preview.png)

<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* Ubuntu 22.04
* ROS2 Humble


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo:
   ```sh
   cd ~ && \
   git clone https://github.com/pvela2017/The-Construct-CheckPoint-10-Robot-Control
   ```
2. Compile the simulation:
   ```sh
   source /opt/ros/humble/setup.bash && \
   cd ~/The-Construct-CheckPoint-10-Robot-Control/ros2_ws && \
   colcon build
   ```
     
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE -->
## Usage
### Local Simulation & Real Robot
1. Launch the simulation:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-10-Robot-Control/ros2_ws/install/setup.bash && \
   ros2 launch rosbot_xl_gazebo empty_simulation.launch.py   # Empty world
   ros2 launch rosbot_xl_gazebo simulation.launch.py         # Maze world
   ```
2. Distance controller:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-10-Robot-Control/ros2_ws/install/setup.bash && \
   ros2 run distance_controller distance_controller
   ```
3. Turn controller:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-10-Robot-Control/ros2_ws/install/setup.bash && \
   ros2 run turn_controller turn_controller
   ```
4. Maze solver:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-10-Robot-Control/ros2_ws/install/setup.bash && \
   ros2 run pid_maze_solver pid_maze_solver
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- RESULTS -->
## Results
[![Demo](https://img.youtube.com/vi/efwq_c8UgqU/0.jpg)](https://www.youtube.com/watch?v=efwq_c8UgqU)

<!-- KEYS -->
## Key Topics Learnt
* PID.
