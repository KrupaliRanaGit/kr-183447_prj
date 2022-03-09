# ADVANCED AMR MAJOR PROJECT PRESENTATION


Goal of this project To navigate the turtlebot to any number of goals published on **/goals** topic without getting collapsed with obstacle and get maximum reward points.

### OVERVIEW ###
   - The goal points will be publish by goal_publisher package. Goals are scattered across the Arena
   - **/goals** have the goal points along with reward points
   - **/gazebo/model_states** gives the position of the robot
   - State is changed between the 2 services, **go_to_point** and **follow_wall**
   -  The model was tested in this Practice Arena
   -  Publishes the angular and linear velocity to this topic to move and rotate the robot. 

 

 ### ALGORITHM USED  

I am using a **BUG_2 algorithm** to reach different goals. Shortest distance and necessary heading from robot's current position to goal position is determine.



### Algorithm Implementation

-  Firstly, robot tries to search goal position and necessary heading towards the goal.
-  if robot finds necessary heading, it moves towards that heading.
-  If obstacle comes in his way, it follows the wall.
-  leaves the wall when it again comes nearer to same heading(m-line). 
-  It leaves the wall and again move towards the goal.
-  If again obstacle comes, above steps are repeated again.

### PROPOSED SOLUTION
 - Sort the goals accordingly value of X and Y co-ordinates and made them in 4 different lists
 - Starting from Quadrant 1, consider (0,0) as the initial position and find distance between the two points for each goal point in that quadrant
 - The distance between two goal points is define by the **hypot function**
 - sorted the list by using **sorted function**, now the first point would be the closest point which is indicated in **tuple**
 - Using this **tuple**, we get the value, (X1, Y1) and store it in a sorted list.
 - At the end of iteration, drop this added value from the sorted list.
 - Now take this (X1, Y1) as the initial position and repeat the same process for the other points in the same quadrant
 - Continue it for other quadrants also by keeping the previous goal as initial position for the remaining points
 - Stop this process when the lists are empty
 - Now the **goal list** will have all the goal points in a sorted order
 - pass this goal list to the program and navigate to them one by one


 


### Problems
One of the biggest challenge is calibrating robot, To set proper parameter so, robot moves easily without collaps. if obstacles in 3 directions, front, left and right and couldn't proceed further inside.

### Solutions
To overcome with this problem, I tried with more and more parameters. The bot to move inside the maze, the front, left and right regions were calibrated by trial and error method.

> These codes are developed as a part of AMR course curriculum , RWU Applied Sciences , 2019
