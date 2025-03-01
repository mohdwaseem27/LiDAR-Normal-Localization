#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import random
import math

def random_pose_publisher():
    # Initialize the ROS node
    rospy.init_node('random_initialpose_publisher', anonymous=True)
    
    # Publisher for /initialpose
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # Wait for the publisher to connect
    rospy.sleep(1)
    
    # Generate random pose within 1 meter radius
    random_x = random.uniform(-1.0, 1.0)  # X between -1m and 1m
    random_y = random.uniform(-1.0, 1.0)  # Y between -1m and 1m
    random_angle = random.uniform(-math.pi / 4, math.pi / 4)  # Angle between -45° and 45°
    
    # Create a PoseWithCovarianceStamped message
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"  # Assuming "map" is the reference frame

    # Set the pose
    pose_msg.pose.pose.position.x = random_x
    pose_msg.pose.pose.position.y = random_y
    pose_msg.pose.pose.orientation.z = math.sin(random_angle / 2)  # Quaternion Z
    pose_msg.pose.pose.orientation.w = math.cos(random_angle / 2)  # Quaternion W

    # Covariance matrix (6x6 identity matrix with some noise)
    pose_msg.pose.covariance = [
        0.25, 0, 0, 0, 0, 0,  # Covariance for x, y, z
        0, 0.25, 0, 0, 0, 0,  # Covariance for y
        0, 0, 0.25, 0, 0, 0,  # Covariance for z
        0, 0, 0, 0.0685, 0, 0,  # Covariance for roll
        0, 0, 0, 0, 0.0685, 0,  # Covariance for pitch
        0, 0, 0, 0, 0, 0.0685  # Covariance for yaw
    ]

    # Publish the message once
    pub.publish(pose_msg)

    # Log message to confirm the pose is being published
    rospy.loginfo(f"Published pose: x={random_x}, y={random_y}, angle={random_angle}")

    # Shutdown the node after publishing once
    rospy.signal_shutdown("Pose published successfully.")

if __name__ == '__main__':
    try:
        random_pose_publisher()
    except rospy.ROSInterruptException:
        pass
