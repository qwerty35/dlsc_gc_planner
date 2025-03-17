# dlsc_gc_planner

This package presents the source code of "Decentralized Trajectory Planning for Quadrotor Swarm in Obstacle-rich Environments with Goal Convergence Guarantee".
The details about this algorithm can be found at the following links.

- **Authors:** Jungwon Park, Yunwoo Lee, Inkyu Jang, and H. Jin Kim from [LARR](http://larr.snu.ac.kr/), Seoul National University
- **Paper:** [Extended version](https://arxiv.org/abs/2209.09447) (NOTE: this version includes the detailed proof that omitted in the ICRA version)
- **Video:** [Youtube](https://youtu.be/PqfdbVfSujA)

%TODO: fix above links

![alt text](images/thumbnail.gif)
%TODO: fix above

## 1. Install
This work is implemented based on C++17. Tested in the ROS Melodic, Ubuntu 18.04

(1) Install ROS Melodic for Ubuntu 18.04 or ROS Noetic for Ubuntu 20.04 (See http://wiki.ros.org/ROS/Installation, desktop-full version is recommended)

(2) Install CPLEX (https://www.ibm.com/products/ilog-cplex-optimization-studio)

(3) Set ROS distro

- ROS Melodic
```
export ROS_DISTRO=melodic
```
- ROS Noetic
```
export ROS_DISTRO=noetic
```

(4) Install dependancies and clone packages
```
sudo apt-get install ros-$ROS_DISTRO-octomap
sudo apt-get install ros-$ROS_DISTRO-octomap-*
sudo apt-get install ros-$ROS_DISTRO-dynamic-edt-3d
cd ~/catkin_ws/src
git clone https://github.com/qwerty35/dlsc_gc_planner.git
```

(5) Before building packages, check CMAKELIST that CPLEX_PREFIX_DIR is indicating the intallation location. For instance, if CPLEX is installed in ```/opt/ibm/ILOG/CPLEX_Studio201```, then CPLEX_PREFIX_DIR should be:
```
set(CPLEX_PREFIX_DIR /opt/ibm/ILOG/CPLEX_Studio201)
```

(6) Build packages
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 2. Demo
- Run single mission
```
source ~/catkin_ws/devel/setup.bash
roslaunch dlsc_gc_planner simulation.launch
```
- Run simulation in the random forest sequentially 
```
source ~/catkin_ws/devel/setup.bash
roslaunch dlsc_gc_planner test_all_forest.launch
```
- Run simulation in the sparse maze sequentially
```
source ~/catkin_ws/devel/setup.bash
roslaunch dlsc_gc_planner test_all_maze_sparse.launch
```
- Run simulation in the dense maze sequentially
```
source ~/catkin_ws/devel/setup.bash
roslaunch dlsc_gc_planner test_all_maze_dense.launch
```
The simulation result will be saved at ```dlsc_gc_planner/log```.

## 3. Configuration
You can configure the simulation setting at the launch, mission files.
- ```launch/simulation.launch```: Mission, octomap, parameters for algorithm
- ```missions/*.json```: Start, goal, dynamical limits of the agent, map size

See the comments in the ```launch/simulation.launch``` and ```missions/readme.txt``` file for more details

Note: If you want to generate the mission file automatically, then use the matlab script in ```matlab/mission_generator```

## 4. Acknowledgment
This work is implemented based on the following packages.

(1) PIBT (https://github.com/Kei18/mapf-IR)

(2) rapidjson (https://rapidjson.org/)

(3) openGJK (https://www.mattiamontanari.com/opengjk/)

(4) convhull_3d (https://github.com/leomccormack/convhull_3d)
