#!/usr/bin/env python

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

LINEAR_SPEED = 0.2
ANGULAR_SPPED = math.pi / 8

#def sub_odom_cb(msg):
def calc_yaw(trans):
    #global yaw
    #orientation_q = msg.pose.pose.orientation
    orientation_q = trans.transform.rotation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

rospy.init_node('ball_follower')

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
#sub_odom = rospy.Subscriber('/odom', Odometry, sub_odom_cb)

yaw = 0.0
direction = 1
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    try:
        robot_trans = tfBuffer.lookup_transform('world', 'base_footprint', rospy.Time())
        ball_trans = tfBuffer.lookup_transform('world', 'ball', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    yaw = calc_yaw(robot_trans)

    ball_x = ball_trans.transform.translation.x
    ball_y = ball_trans.transform.translation.y
    diff_x = ball_x - robot_trans.transform.translation.x
    diff_y = ball_y - robot_trans.transform.translation.y

    print "diffx " + str(diff_x)
    print "diffy " + str(diff_y)
    print "yaw: " + str(yaw)
    
    main_angle = math.atan2(diff_y, diff_x)
    angle_difference = (main_angle - yaw)
    distance = math.sqrt(diff_y*diff_y + diff_x*diff_x)

    twist = Twist()
    
    if angle_difference < 0:
        direction = - 1
    else:
        direction = 1

    if (-math.pi/16 < angle_difference < math.pi/16):
        twist.linear.x = LINEAR_SPEED
        twist.angular.z = ANGULAR_SPPED * direction
    else:
        twist.angular.z = ANGULAR_SPPED * direction

    if distance <= 0.3:
        # print "CAUGHT YOU"
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    # else:
        # print "Don't run away..."

    pub_vel.publish(twist)

    rate.sleep()




