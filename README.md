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

For the first state, _final_ui.py_ node requests _my_srv_ for a random target position between the range of 1 to 6. Then, the main node publishes the target positions to 

<!--


In the first choice, the main node sends a service request to the tiserver and gets a random target position. Then, it publishes this position to the /move_base/goal and then continuously checks the status of the goal by subscribing to /move_base/status topic. Once, the status indicates reaching the target. The interface asks the user to enter another request.
In the second choice, the main node asks the user to choose one out of 6 possible target positions, and publishes it to /move_base/goal as in the previous choice.
In the third choice, the main node sends a request to the wall_follower service in order for the robot to start following the walls. The user can enter another request anytime during following the walls.
In the fourth choice, the main node stops wall following and publishes zero velocity command to /cmd_vel, and the robot stops moving.



In final_assignment package we have multiple components such as house.world file which is our simulation environment. For execution purposes of this simulation environment we use simuation_gmapping.launch file which present in the launch folder. Besides this we also have Sim.rviz file in config folder. Since we our using move_base technique for localization and mapping the simulation environment, therefore we also a have move_base.launch file in launch folder.
To accommodate the 'follow_wall' functionality of the robot, we have a wall_follow_service_m.py file in the scripts folder which allow robot to move continuously along the wall. Beside this we have a main file named 'f_a_user_interface.py' (f_a_user_interface is stands for final_assignment user interface) which handles all the user interface related functionalities along with merging other nodes. The details of the functionalities performed in this node is explain below.
This node perform following functionalities:
1.	In this node we have initialized two service clients one for 'f_a_target_server' in order to receive random target index and one for robot wall follower script which basically allows to robot to follow walls as mentioned above.
2.	There is one subscriber initialized to '/move_base/status' topic of type GoalStatusArray which we use to see weather the robot has reached its target or not.
3.	There are two publishers also initialized in this node one to publish the robot's new target values in the topic '/move_base/goal' and the second one to publish robot's velocities in the '/cmd_vel' topic.
4.	The user interface is designed to allow user to decide which state of the robot they want execute out of total four. a) In the first state, the robot randomly choose the new target position values from a set of six predefined positions. Once the robot has reached to its target position the user has the option to keep it this state or change it to something else. In order to achieve this behaviour we first send request to f_a_target_server for random target index then by using this index value we set the new target position for robot. Once the new target position has been set we publish these values to '/move_base/status' topic.
b) In the second state, the robot ask the user to choose the new target position from the list of six predefined positions. Again, once the robot has reached to its target position the user will have the option change the state or keep it as it is. We achieve this behaviour of robot by first finalizing the robot's new target values using user's input and then publishing it on '/move_base/status' topic.
c) In the third state, the robot follow the simulated environment wall using the wall follower service client d) In the fourth state, the robot stops in its position. We do it by publishing the robot's velocity values as zero in '/cmd_vel' topic.
-->

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

