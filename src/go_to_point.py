#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from tf import transformations
from std_srvs.srv import *
from goal_publisher.msg import PointArray
from math import hypot
from operator import itemgetter

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3
#dist_precision_ = 0.5

goal_list = []
end_flag = 0

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_ModelStates(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose[1].position
    
    # yaw
    quaternion = (
        msg.pose[1].orientation.x,
        msg.pose[1].orientation.y,
        msg.pose[1].orientation.z,
        msg.pose[1].orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

     
def Sort_Points(list_points, start):
    sorted_list = [] 
    dist = []
    p1 = [start[0], start[1]]
    counter = len(list_points)

    while (counter > 0):
        for j in range(len(list_points)):
            p2 = [list_points[j][0], list_points[j][1], list_points[j][2], list_points[j][3]]
            distance = hypot(p1[0] - p2[0], p1[1] - p2[1])    
            dist.append(str(distance))
            
        sorted_dist = sorted(dist,key=float)
        print "sorted_dist: "
        print sorted_dist
        ind = dist.index(sorted_dist[0])
        p1 = [list_points[ind][0], list_points[ind][1], list_points[ind][2], list_points[ind][3]]
        sorted_list.append(tuple(p1))
        list_points.remove(tuple(p1))
        counter = counter - 1       
        del dist[:]

    print "sorted_list: "
    print sorted_list
    return sorted_list


def callback_goals(msg):
    global desired_position_
    global initial_position_
    global goal_list
    global end_flag
    global next_flag

    List_quad1 = []
    List_quad2 = []
    List_quad3 = []
    List_quad4 = []

    length = len(msg.goals)



    if not goal_list:
         for i in range(length):
             if (msg.goals[i].x >= 0) and (msg.goals[i].y >= 0):
                goals_quad1 = (msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward)
                List_quad1.append(goals_quad1)

             elif (msg.goals[i].x < 0) and (msg.goals[i].y >= 0):
                goals_quad2 = (msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward)
                List_quad2.append(goals_quad2)

             elif (msg.goals[i].x < 0) and (msg.goals[i].y < 0):
                goals_quad3 = (msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward)
                List_quad3.append(goals_quad3)

             elif (msg.goals[i].x >= 0) and (msg.goals[i].y < 0):
                goals_quad4 = (msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward)
                List_quad4.append(goals_quad4)

         start = [0,0]
         sorted_quad1 = Sort_Points(List_quad1,start)
         start = [sorted_quad1[len(sorted_quad1) -1][0], sorted_quad1[len(sorted_quad1) -1][1]]
         sorted_quad2 = Sort_Points(List_quad2,start)
         start = [sorted_quad2[len(sorted_quad2) -1][0], sorted_quad2[len(sorted_quad2) -1][1]]
         sorted_quad4 = Sort_Points(List_quad4,start)
         start = [sorted_quad4[len(sorted_quad4) -1][0], sorted_quad4[len(sorted_quad4) -1][1]]
         sorted_quad3 = Sort_Points(List_quad3,start)

         goal_list = sorted_quad1 + sorted_quad2 + sorted_quad4 + sorted_quad3

         rospy.loginfo('goal_list: ')
         rospy.loginfo(goal_list)

    if not end_flag:
        desired_position_.x = goal_list[0][0]
        desired_position_.y = goal_list[0][1]


    if (abs(position_.x - desired_position_.x) <= dist_precision_) and (abs(position_.y - desired_position_.y) <= dist_precision_):
        end_flag += 1
        next_flag = True

    if end_flag and (end_flag < len(goal_list)):
        desired_position_.x = goal_list[end_flag][0]
        desired_position_.y = goal_list[end_flag][1]      

def change_state(state):
    global state_
    state_ = state
   
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    rospy.loginfo("fix_yaw")
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.4 if err_yaw > 0 else -0.4
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)

def go_straight(des_pos):
    global yaw_, pub, yaw_precision_, state_
    rospy.loginfo("go_straight_ahead")
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = 0.0 if err_yaw > 0 else -0.0
        pub.publish(twist_msg)
    else:
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

def done():
    global state_ 
    state_ = 0 # for moving robot to the next goal (repeate with fix_yaw)
    rospy.loginfo("Inside Done")
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_goals = rospy.Subscriber('/goals',PointArray, callback_goals)
    sub_model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, clbk_ModelStates)
    
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
   try:
       main()
   except rospy.ROSInterruptException:
   		 pass
