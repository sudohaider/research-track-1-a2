# Research Track - Final Assignment
##### Submitted by Muhammad Ali Haider Dar (S5046263)
This is a ROS architecture for the control of a mobile robot in the Gazebo environment. The software relies on the move_base and gmapping packages for localizing the robot and plan the motion. The architecture gets the user request, and lets the robot execute one of the pre-defined behaviors accordingly, along with Simulataneous Localization and Mapping (SLAM), path planning, and collision avoidance.

The program requests user input on the following states or choices:
1. Robot moves randomly in the environment, by choosing 1 out of 6 possible target positions:
[(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)].
2. Program asks the user for the next target position, checking that the position is one of the possible six target positions, and the robot reaches it.
3. Robot starts following the external walls.
4. Robot stops in the last position.

If the robot is in state 1, or 2, the system waits until the robot reaches the position in order to switch to the state 3 and 4.

## Description of Files

### Simulator: _Gazebo_

Gazebo is the 3D simulator for ROS. Further documentation about Gazebo can be read [_here_](http://gazebosim.org/).

### Visualizer: _rviz_

_rviz_ is a 3D tool for ROS Visualization. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions. Further documentation about _rviz_ can be read [_here_](http://wiki.ros.org/rviz).

### Controller: _final_assignment_
The controller package _final_assignment_ contains the scripts, launch files and other dependencies used to simulate the 3D environment and move the robot in it. The simulation_gmapping.launch file launches the house.world file environment. The main node _final_ui.py_ contains the entire control structure for the mobile robot simulation. 

1. For the first state, _final_ui.py_ node requests _my_srv_ for a random target position between the range of 1 to 6. Then, the main node publishes the target positions to `/move_base/goal` and check thes the status of `goal` by subscribing to the topic `/move_base/status`. When the robot reaches the target and the status displays it in the node, the main node requests the user to input again.
2. For the second state, the user chooses one out of six possible target positions (as before) and publishes it to `/move_base/goal`.
3. For the third state, the wall_follower service is utilized through initialization of a service client to allow the robot to follow the walls. The interface also allows the user to enter the same or different request at any point in this state. 
4. For the fourth state, the node stops all actions and stops the robot by publishing commands of zero velocity in topic `/cmd_vel`. Same as in state 3, the interface allows the user to enter the same or different request at any point in this state.

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

