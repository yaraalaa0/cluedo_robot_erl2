#! /usr/bin/env python

"""
.. module:: go_to_point_action.py
   :platform: Unix
   :synopsis: this file is an implementation of go to point action server node
   
.. moduleauthor:: Carmine Recchiuto - Yara Abdelmottaleb
 
This node implements the go to point action server node. 
When an action request is received with a goal position, it drives the robot towards the goal.
It drives the robot in three steps:
- rotating the robot around its z-axis until it is oriented towards the goal point
- going straight ahead until it reaches the goal point
- rotating the robot around its z-axis until it is oriented in the goal orientation
During the three steps, if a cancel request is received through the action server, 
the goal is aborted and the robot stops.
 
Subscribes to:
   /odom
 
Publishes to:
   /cmd_vel
 
Services:
   /go_to_point_ac
  
"""
# import ros stuff
import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import erl2.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0  ## current robot's yaw angle
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0  
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publisher
pub = None

# action_server
act_s = None

# callbacks


def clbk_odom(msg):
    """
    this is the callback function of /odom subscriber
    This function receives periodically the current odometry of the robot. 
    It stores the position in the global variable position_
    It converts the orientation from quaternion to Euler representation and stores the yaw angle in global variable yaw_
    
    Args:
      msg(Odomoetry): the current odometry of the robot received through the topic /odom. 
      
    """
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    this is change_state function of the robot
    This function takes as input the current state value of the robot. 
    It updates the global variable state_ accordingly with the current new state
    
    Args:
      state(int): the current state value of the robot (0, 1, 2)
    
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    This function normalizes the angle to be in the range from -pi to pi
    This function takes as input an angle expressed in randians. It checks if this angle is in the range of -pi to pi
    If it is not, the function normalizes the angle to be in the range.
    
    Args:
      angle(float): the requested angle to be normalized
    
    Returns:
      the normalized angle
     
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """
    This function is responsible for orienting the robot's yaw angle towards the goal point
    This function takes as input the goal point. It calculates the error angle needed to be oriented towards the goal.
    It, then, publishes angular twist msg to the robot to be rotated around the z-axis. 
    It checks that the angular velocity msg is in the range between lower and upper bounds.
    If the error yaw angle to the goal is less than the precision threshold, it changes the state to 1 to indicate finishing the rotation step
    
    Args:
      des_pos(Point): the desired position of the goal
      
    """
    global yaw_, pub, yaw_precision_2_, state_
    ## Check at what stage the robot is. Based on that, compute the deired yaw
    if state_ == 0:
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    elif state_ == 2:
        if des_pos.x == 2.4 and des_pos.y == 0:
            desired_yaw = 0
        elif des_pos.x == 0 and des_pos.y == 2.4:
            desired_yaw = math.pi/2
        elif des_pos.x == -2.4 and des_pos.y == 0:
            desired_yaw = math.pi
        elif des_pos.x == 0 and des_pos.y == -2.4:
            desired_yaw = -math.pi/2
        elif des_pos.x == 0 and des_pos.y == 0:
            desired_yaw = 0
    
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        if state_ == 0:
            #print ('Yaw error: [%s]' % err_yaw)
            change_state(1)
        elif state_ == 2:
            change_state(3)


def go_straight_ahead(des_pos):
    """
    This function is responsible for driving the robot straight towards a goal position
    This function takes as input the desired goal position. It calcualates the error in distance between the current position and the goal.
    If the distance error is greater than the distance precision threshold, it publishes a Twist msg with a linear velocity to the robot. 
    When the distance error is less than the threshold, it changes the state to 2 to indicate finishing the go_straight_ahead step
    
    Args:
      des_pos(Point): the desired goal position
    
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d
        
        twist_msg.angular.z = kp_a*err_yaw
        
        pub.publish(twist_msg)
    else:
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    """
    This function is responsible for stopping the robot
    This function is responsible for stopping the robot after reaching the goal, or when the goal is cancelled.
    It publishes a zero velocity msg to the robot to stop it
    
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def planning(goal):
    """
    This is the callback function of /reaching_goal action service
    This function takes as input the goal request (x, y, theta). It initializes the robot state to 0.
    Then, it loops until the goal position is reached or it is preempted.
    In each loop, it does the following:
    - it checks if a cancel request has been sent. If yes, it cancels the goal, stops the robot, and break from the loop.
    - it publishes the current robot's position as a feedback of the action service
    - it checks the current state of the robot and based on each state, it calls the appropriate function to drive the robot towards the goal.
    - When the goal is reached, it stops the robot and sets the action result
    
    Args:
      goal(Point): the requested goal position
      
    Returns:
      always true
      
    """
    global state_, desired_position_
    global act_s
    

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y

    state_ = 0
    rate = rospy.Rate(5)
    success = True

    feedback = erl2.msg.PlanningFeedback()
    result = erl2.msg.PlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            feedback.stat = "Fixing the final yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 3:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)


def main():
    """
    This is the main function of the node
    It initializes the node handle, the publisher to /cmd_vel topic
    the subscriber to /odom, and the action server of /go_to_point_ac service.
    It assigns the function execute_action as a callback function to the action server
    
    """
    global pub, active_, act_s
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/reaching_goal', erl2.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
