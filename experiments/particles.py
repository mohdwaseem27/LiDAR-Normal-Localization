# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseArray
# import numpy as np

# class ParticlePoseRMSEChecker:
#     def __init__(self):
#         rospy.init_node('particle_pose_rmse_checker')
        
#         self.odom_pose = None
#         self.particle_within_range = False
        
#         # Subscribe to particle pose array topic
#         rospy.Subscriber('/mcl_optimal_node/particles', PoseArray, self.particles_callback)
        
#         # Subscribe to odometry topic
#         rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
#     def odom_callback(self, odom_msg):
#         # Store odometry pose
#         self.odom_pose = odom_msg.pose.pose
        
#     def get_odom_pose(self):
#         # Function to get odometry pose
#         return self.odom_pose
        
#     def particles_callback(self, particles_msg):
#         if self.odom_pose is None:
#             rospy.logwarn("Odometry data is not available yet.")
#             return
        
#         # Extract particle poses from PoseArray message
#         particle_poses = particles_msg.poses
        
#         for pose in particle_poses:
#             # Calculate RMSE between particle pose and odometry pose
#             rmse = np.sqrt((pose.position.x - self.odom_pose.position.x)**2 +
#                            (pose.position.y - self.odom_pose.position.y)**2 +
#                            (pose.position.z - self.odom_pose.position.z)**2)
#             # Check if any particle is within the range of 1 meter
#             if rmse <= 1:
#                 self.particle_within_range = True
#                 break
        
#         # If no particle is within the range of 1 meter, print True and stop the node
#         if not self.particle_within_range:
#             rospy.loginfo("True")
#             rospy.signal_shutdown("No particles within the range of 1 meter.")

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     particle_pose_checker = ParticlePoseRMSEChecker()
#     particle_pose_checker.run()


#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys

# Global variables to store the latest messages
odom_pose = None
amcl_pose = None

def odom_callback(msg):
    global odom_pose
    odom_pose = msg.pose.pose  # Extract the odom pose

def amcl_pose_callback(msg):
    global amcl_pose
    amcl_pose = msg.pose.pose  # Extract the amcl pose

def calculate_distance(pose1, pose2):
    # Compute Euclidean distance

    return math.sqrt(
        (pose1.position.x - pose2.position.x) ** 2 +
        (pose1.position.y - pose2.position.y) ** 2
    )

def main():
    global odom_pose, amcl_pose

    rospy.init_node('amcl_odom_distance_checker', anonymous=True)

    # Subscribers
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if odom_pose is not None and amcl_pose is not None:
            distance = calculate_distance(odom_pose, amcl_pose)
            if distance <= 1.0:
                print(True)
            else:
                print(False)
            rospy.signal_shutdown("Check completed.")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
