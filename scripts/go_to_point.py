#! /usr/bin/env python -tt

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import rt2_assgnment1.msg
from std_msgs.msg import String, Float64

# robot state variables
position_ = Point()
pose_=Pose()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

desired_position_= Point()
desired_position_.z=0
success = False
Vel=Twist()
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

#action server
act_s=None

def clbk_vel(msg):
    global Vel
    Vel.linear.x=msg.linear.x
    Vel.angular.z=msg.angular.z

def clbk_odom(msg):
    global position_
    global pose_
    global yaw_
	
    position_ = msg.pose.pose.position

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Vel.angular.z
    pub_.publish(twist_msg)
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = des_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Vel.linear.x
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = Vel.linear.x
        twist_msg.angular.z = Vel.linear.z*err_yaw
        pub_.publish(twist_msg)
    else: 
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    global Vel
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Vel.angular.z
    pub_.publish(twist_msg)
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()
    
def go_to_point(goal):
    global state_, desired_position_, act_s, success
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.position.z
    change_state(0)
    while True:
        #checking if the client is requested to cancel the goal
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            act_s.set_preempted()
            success=False 
            break
        elif state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    return True

def main():
    global pub_, active_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_linVel = rospy.Subscriber('/vel', Twist, clbk_vel)
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.GotopointAction, go_to_point, auto_start=False)
    act_s.start()
    rospy.spin()

if __name__ == '__main__':
    main()
