#! /usr/bin/env python
import sys
import tf
import math
import rosbag

trans_drift_per_m = 0.9
rot_drift_per_rad = 0.9
rot_drift_per_m = 0.05
x = y = yaw = 0
t_last = 0
first_odom_msg = True

if len(sys.argv) == 2:
    with rosbag.Bag('out.bag', 'w') as outbag:
        print ("Cleaning... ")
        for topic, msg, t in rosbag.Bag(str(sys.argv[1])).read_messages():
            if topic != "/tf" and topic != "/tf_static":
                if topic == "/ground_truth_odom":
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
                        
                        outbag.write(topic, msg, msg.header.stamp)
                        msg.pose.pose.position.x = x
                        msg.pose.pose.position.y = y
                        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                        msg.pose.pose.orientation.x = quat[0]
                        msg.pose.pose.orientation.y = quat[1]
                        msg.pose.pose.orientation.z = quat[2]
                        msg.pose.pose.orientation.w = quat[3]
                        outbag.write("/odom", msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, msg.header.stamp)
    print ('Cleaned ' + str(sys.argv[1]) + '.bag')

