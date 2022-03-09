#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from tf import transformations
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from math import hypot
from operator import itemgetter
import time


# import ros service
from std_srvs.srv import *
from goal_publisher.msg import PointArray
import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
position_ = Point()
initial_position_ = Point()
dist_precision_ = 0.3


initial_position_.x = 0.01
initial_position_.y = 0.01
initial_position_.z = 0
desired_position_ = Point()
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
goal_list = []
end_flag = 0
next_flag = False
reward_point = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_ModelStates(msg):
    global position_, yaw_

    # position
    position_ = msg.pose[1].position #position of turtlebor = 1

    # yaw
    quaternion = (
        msg.pose[1].orientation.x,
        msg.pose[1].orientation.y,
        msg.pose[1].orientation.z,
        msg.pose[1].orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(msg.ranges[269:314]),
        'fright': min(msg.ranges[315:349]),
        'front':  min(msg.ranges[0:9]) +  min(msg.ranges[350:359]),
        'fleft':  min(msg.ranges[10:39]),
        'left':   min(msg.ranges[40:89]),
    }



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
    global reward_point

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
         print "goal_list: "
         print goal_list

    if not end_flag:
        initial_position_.x = position_.x
        initial_position_.y = position_.y 
        desired_position_.x = goal_list[0][0]
        desired_position_.y = goal_list[0][1]


    if (abs(position_.x - desired_position_.x) <= dist_precision_) and (abs(position_.y - desired_position_.y) <= dist_precision_):
        rospy.loginfo("Goal " + str(end_flag+1) + " REACHED SUCCESSFULLY ")
        reward_point += goal_list[end_flag][3]
        rospy.loginfo("reward_point" + str(reward_point))
        time.sleep(1)
        end_flag += 1
        next_flag = True

    if end_flag and (end_flag < len(goal_list)):
        initial_position_.x = goal_list[end_flag-1][0]
        initial_position_.y = goal_list[end_flag-1][1]
        desired_position_.x = goal_list[end_flag][0]
        desired_position_.y = goal_list[end_flag][1]


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # here goes the equation
    #rospy.loginfo("p2.x" + str(p2.x))
    #rospy.loginfo("p2.y" + str(p2.y))
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance_to_line


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global distance, closest_distance, closest
    global count_state_time_, count_loop_
    global next_flag
    rospy.init_node('bug2')

    rate = rospy.Rate(20)
    sub_goals = rospy.Subscriber('/goals',PointArray, callback_goals)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_model = rospy.Subscriber('/gazebo/model_states', ModelStates, clbk_ModelStates)



    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
#    rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)



    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if next_flag:
           rospy.loginfo("change flag done")
           rospy.loginfo("desired_position_.x" + str(desired_position_.x))
           rospy.loginfo("desired_position_.y" + str(desired_position_.y))
           change_state(0)
           next_flag = False

        distance_position_to_line = distance_to_line(position_)

        if state_ == 0:
            #rospy.loginfo(regions_['front'])
            if regions_['front'] > 0.8 and regions_['front'] < 1.3 :
                rospy.loginfo("Changing to wall follower")
                change_state(1)

        elif state_ == 1:
            if count_state_time_ > 5 and \
               distance_position_to_line < 0.1:
                change_state(0)

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        #rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        rate.sleep()

if __name__ == "__main__":
   try:
       main()
   except rospy.ROSInterruptException:
   		 pass
