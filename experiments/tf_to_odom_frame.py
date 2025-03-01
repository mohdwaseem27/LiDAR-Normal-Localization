#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs  # Import this for do_transform_pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class PoseTransformer:
    def __init__(self):
        rospy.init_node('pose_transformer', anonymous=True)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the amcl_pose topic in the map frame
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Create a publisher for the transformed pose in the odom frame
        self.pose_publisher = rospy.Publisher('/robot_pose_transformed', Odometry, queue_size=10)

    def pose_callback(self, amcl_pose):
        try:
            # Get the transform from map to odom
            transform = self.tf_buffer.lookup_transform('odom', 'map', rospy.Time())

            # Create an Odometry message
            odom_msg = Odometry()
            odom_msg.header.frame_id = 'odom'
            # Transform the pose to the odom frame using tf2_geometry_msgs
            odom_msg.pose.pose = tf2_geometry_msgs.do_transform_pose(amcl_pose.pose, transform).pose

            # Publish the transformed pose as Odometry
            self.pose_publisher.publish(odom_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform pose: {}'.format(e))

if __name__ == '__main__':
    try:
        pose_transformer = PoseTransformer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
