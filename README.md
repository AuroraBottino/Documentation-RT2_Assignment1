# RT1 - Assignment 2 - ROS Simulator 
RT1 - Assignment 2 - ROS Simulator
**Professor: Carmine Tommaso Recchiuto, Student: Aurora Bottino (s4814504)**

# Assignment Description

Starting from the package assignment_2_2022 (https://github.com/CarmineD8/assignment_2_2022), which provides an implementation of an action server that moves a robot in the environment by implementing the bug0 algorithm, the following project creates a new package in which three nodes are developed:

(a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom.

(b) A service node that, when called, prints the number of goals reached and cancelled.

(c) A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed. Use a parameter to set how fast the node publishes the information. 

In addition it is required to create a launch file in order to start the whole program.
# Nodes
There are six nodes in the package:

**bug_as.py**: the action server node that calls the services to move the robot to the goal position

**go_to_point_service.py**: the service node to move the robot to the goal position

**wall_follow_service.py**: the service node to avoid obstacles

and then I implemented the three nodes requested:

**print_info.py**: the node that subscribes to the robot’s position and velocity from the /Position_velocity as a custom message and prints the distance of the robot from the target and the robot’s average speed with a frequency setted as a parameter in the lauch file.

**service.py**: the service node that prints the number of goals reached and cancelled.

**user_input.py**: the action client node which allows the user to set a goal position with coordinates x and y, or to cancel it. The node also publishes the robot position and velocity as a custom message on the /Position_velocity topic, by relying on the values published on the topic /odom. The structure of the node is described in the following flowchart:

![Screenshot (250)](https://user-images.githubusercontent.com/114871147/214496968-e685a5b9-5ef7-4fc5-88f3-681b098d5e71.png)


# Install and run
1) Start the master: **roscore &**
3) Go inside the src folder of ROS workspace and clone the assignment folder: **https://github.com/AuroraBottino/assignment_2_2022.git**
4) Run the command: **catkin_make**
5) Install: **sudo apt-get install xterm**
6) Run: **roslaunch assignment_2_2022 assignment1.launch**.
If everything works correctly, the following windows will be opening:

![research](https://user-images.githubusercontent.com/114871147/214268431-e0790db8-2d36-41f7-a6db-b2709b7292e8.png)

plus the user_input (where the user enters the coordinates) and the print_info windows. 
# Possible improvements:
Some improvements that can be done are:
1) Having a more clear view of the goal position the robot has to reach: this could be done adding a graphical marker or creating a function that allows us to recognize the size of the arena, in order to bound the coordinates. 
2) Improving the bug0 algorithm: sometimes the robot reaches the wall and is not able to move anymore or it chooses itself which direction it's better to take to reach the goal.
