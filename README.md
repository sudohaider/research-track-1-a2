# Research Track - Final Assignment
##### Submitted by Muhammad Ali Haider Dar (S5046263)
A ROS architecture for the control of the robot in the environment. The software relies on the move_base and gmapping packages for localizing the robot and plan the motion. The architecture gets the user request, and lets the robot execute one of the pre-defined behaviors accordingly, along with Simulataneous Localization and Mapping (SLAM), path planning, and collision avoidance.

## Description of Files

### Simulator: _Gazebo_

Gazebo is the 3D simulator for ROS. Further documentation about Gazebo can be read [_here_](http://gazebosim.org/).

### Visualizer: _rviz_

_rviz_ is a 3D tool for ROS Visualization. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions. Further documentation about _rviz_ the package can be read [_here_](http://wiki.ros.org/rviz).

### Controller: _final_assignment_

The controller package _assignment1_pkg_ contains the C++ file **controller.cpp** which contains the source code for controlling the robot. The functionality is called through the node _assignment1_node_.

Position callback function is implemented for subscribing to the topic `/odom` from nav_msgs/Odometry. The function initializes and calls service to acquire values of new position of robot. The destination is reached when the distance between the current position and destination is less than 0.1, and the function requests for new positions. The linear velocities are published on topic _cmd_vel_ according to the formula **vel_x = k (new_x - current_x)**, where k = 1 in this case. A message is published on topic _assignment1/position_ containing the name of message, _x_, _y_, _new_x_ and _new_y_ coordinates.

### Server: _my_srv_

The server package _my_srv_ contains the C++ file **final_server.cpp** which contains the source code for generating random integer within a specified range and advertising it over the node `/final`. It uses a custom message which requests two integers namely _min_ and _max_, and returns one random integer _target_index_ within this range in response.

## Instructions for Running the Project

### Getting Started

The following steps will help prepare the necessary environment and dependencies to run this project.

1. Open command line terminal and navigate to the the local workspace using `cd /<name_of_workspace>/src` command.

3. Clone the remote repository of assignment in your local workspace using the command:
```
git clone https://github.com/alihaidersays/ResearchTrack_FinalAssignment.git
```

3. Build the file using `catkin_make` in the root folder of your workspace.

4. Execute the command `rospack profile` to read and parse the .xml for each package and assemble a complete dependency tree for all packages.

### Running the Program

The following steps will run the simulator along with the controller nodes.

1. **Gazebo** is the 3D simulator while **RViz** is the 3D visualization tool for ROS. In the command line, launch Gazebo and RViz by executing the following command:
```
roslaunch final_assignment simulation_gmapping.launch
```

2. In a new command line tab, run the following command:
```
roslaunch final_assignment move_base.launch
```

3. In a new command line tab, run the following command:
```
rosrun final_assignment wall_follow_service_m.py
```

4. In a new command line tab, run the following command:
```
rosrun my_srv final_server
```

5. In a new command line tab, run the following command:
```
rosrun final_assignment final_ui.py
``` 

6. The information getting published in the topics can be printed on the command line using the 'echo' command. Run the following command:
```
rostopic echo /move_base/status
```

7. To display a graph of what's going on in the system, run the following command in a new command line tab: 
```
rosrun rqt_graph rqt_graph
```
The computational graph of the system _rosgraph_final_assignment.png_ can be found below:

![alt text](https://github.com/alihaidersays/ResearchTrack_FinalAssignment/blob/main/rosgraph_final_assignment.png)

## Documentation

The documentation for the project can be found in HTML format inside _docs_ folder. Doxygen tool was used for the documentation of C++ files.

