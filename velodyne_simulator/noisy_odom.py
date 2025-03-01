#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry

trans_drift_per_m = 0.9
rot_drift_per_rad = 0.9
rot_drift_per_m = 0.05
x = y = yaw = 0
t_last = 0
first_odom_msg = True

odom_pub = None  # Declare odom_pub as a global variable

def odom_callback(msg):
    global x, y, yaw, t_last, first_odom_msg, odom_pub

    if first_odom_msg:
        first_odom_msg = False
        t_last = msg.header.stamp.to_sec()
    else:
        t_diff = msg.header.stamp.to_sec() - t_last
        t_last = msg.header.stamp.to_sec()
        x_diff = t_diff * msg.twist.twist.linear.x
        y_diff = t_diff * msg.twist.twist.linear.y
        yaw_diff = t_diff * msg.twist.twist.angular.z

        x += trans_drift_per_m * (x_diff * math.cos(yaw) - y_diff * math.sin(yaw))
        y += trans_drift_per_m * (x_diff * math.sin(yaw) + y_diff * math.cos(yaw))
        yaw += rot_drift_per_rad * yaw_diff + rot_drift_per_m * (x_diff + y_diff)

        # Publish modified odometry message
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_pub.publish(odom_msg)

def main():
    global odom_pub

    rospy.init_node('noisy_odom')

    # Create a publisher for the modified odometry messages
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Subscribe to the original odometry topic
    rospy.Subscriber('/ground_truth_odom', Odometry, odom_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

