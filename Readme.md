# AMR Final Project

The repository contains an control strategy which will make robot to reach different goals published on /goals topic using Bug2 algorithm involving obstacle avoidance and wall following.

### Getting Started ###

  - Launch Gazebo Empty World with turtlebot3 as robot model using roslaunch turtlebot3_gazebo turtlebot3_empty_world
  - Launch the arena by using the command
	rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/<your tier4 repo>/final_project_practice/model.sdf -sdf -x -0.5 -y 2 -model mini_project
  - Launch the ROS package for the package using the command
	roslaunch  kr_183447_prj start.launch

# Pre-requisites

1) Ubuntu OS
2) ROS Kinetic
3) Atom
4) Gazebo Simulator
5) Terminator
6) Config for Goals Publisher
7) Arena Model

# General Description
Goal of this project - To navigate the turtlebot to any number of goals published on /goals topic without getting collapsed with obstacle and get maximum reward points.

Used Topic and Service -
    /goals - This subscribed topic is for getting goal positoins
    /scan - This is a subscribed topic to get Laser Scan Data
	/gazebo/model_states - Subscribed to this topic to get current position of robot
    /cmd_vel - Publishes the angular and linear velocity to this topic to move and rotate the robot.
    
# Algorithm 
    I am using a BUG2 algorithm to reach different goals. Shortest distance and necessary heading from robot's current position to goal position is determine.
    
    Thereafter robot moves towards that heading and if some obstacles comes in pathway, it tries to follow the obstacle without moving farther from necessary(m-line) heading.
    
    Using this algorithm, control strategy is developed to navigate the robot to goal positions.
    For improving robot performance, I am sorting goals quadrat wise for saving time.  
    
# Algorithm Implementation

In 4 steps bug2 algorithm implemented
1. move to obstacle or Goal
2. Find wall
3. wall Following
4. Turn left
5. Go to Point

Firstly, it tries to search goal position and necessary heading towards the goal. After rotating, if it finds necessary heading, it moves towards that heading. If obstacle comes in 
 his way, it follows the wall. Then, leaves the wall when it again comes nearer to same heading(m-line). It leaves the wall and again move towards the goal. If again obstacle comes, 
 above steps are repeated again.
 
# Problems
  One of the biggest challenge is callibrating robot, To give proper parameter so, robot moves easily without collaps.

# Solution
  To overcome with this problem, I tried with more and more parameters. 
