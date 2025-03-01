# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Quaternion
# from tf.transformations import euler_from_quaternion
# from math import pi, fabs

# class MoveRotateMove:
#     def __init__(self):
#         rospy.init_node('move_rotate_move_node', anonymous=True)
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
#         self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         self.rate = rospy.Rate(10)  # 10 Hz

#         self.target_yaw = pi / 2  # 90 degrees in radians
#         self.yaw_tolerance = 0.02  # 2.86 degrees in radians
#         self.linear_speed = 0.3  # m/s
#         self.angular_speed = 0.2  # rad/s
#         self.distance_to_travel_y = 6.0  # meters for y-axis movement
#         self.distance_to_travel_x = 21.0  # meters for x-axis movement
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0

#         # PID parameters for rotation control
#         self.Kp_rotation = 1.0
#         self.Ki_rotation = 0.01
#         self.Kd_rotation = 0.5
#         self.prev_error_rotation = 0.0
#         self.error_integral_rotation = 0.0

#         # PID parameters for position control
#         self.Kp_position = 1.1
#         self.Ki_position = 0.0
#         self.Kd_position = 0.6
#         self.prev_error_position = 0.0
#         self.error_integral_position = 0.0

#     def odom_callback(self, odom_msg):
#         orientation_quaternion = odom_msg.pose.pose.orientation
#         orientation_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
#         _, _, self.current_yaw = euler_from_quaternion(orientation_list)
#         self.current_x = odom_msg.pose.pose.position.x
#         self.current_y = odom_msg.pose.pose.position.y

#     def rotate(self, target_yaw):
#         print("here")
#         while not rospy.is_shutdown():
#             print(target_yaw *180 / pi, self.current_yaw * 180 / pi, fabs(target_yaw - self.current_yaw) * 180/pi)
#             if fabs(target_yaw - self.current_yaw) <= self.yaw_tolerance:
#                 rospy.loginfo("Reached target orientation.")
#                 break
#             else:
#                 twist_msg = Twist()
#                 twist_msg.angular.z = self.angular_speed
#                 self.vel_pub.publish(twist_msg)
#                 self.rate.sleep()

#         # Stop the rotation
#         stop_msg = Twist()
#         self.vel_pub.publish(stop_msg)

#     def move_forward(self, dir, distance_to_travel, pid_dir = None):
#         if dir == "y":
#             self.initial_y = self.current_y
#             self.initial_x = self.current_x
#             while not rospy.is_shutdown():
#                 if fabs(self.current_y - self.initial_y) >= distance_to_travel:
#                     rospy.loginfo("Reached destination.")
#                     break
#                 error = self.current_x - self.initial_x
#                 twist_msg = Twist()
#                 twist_msg.linear.x = self.linear_speed
#                 if fabs(error) >= 0.01:  # Tolerance for position control
#                     angular_speed = self.Kp_position * error + self.Ki_position * self.error_integral_position + self.Kd_position * (error - self.prev_error_position)
#                     self.error_integral_position += error
#                     self.prev_error_position = error
#                     if pid_dir == "minus":
#                         twist_msg.angular.z = -angular_speed
#                     else:
#                         twist_msg.angular.z = angular_speed
#                 self.vel_pub.publish(twist_msg)
#                 self.rate.sleep()
#         if dir == "x":
#             self.initial_x = self.current_x
#             self.initial_y = self.current_y
#             while not rospy.is_shutdown():
#                 print("should be here")
#                 if fabs(self.current_x - self.initial_x) >= distance_to_travel:
#                     rospy.loginfo("Reached destination.")
#                     break
#                 error = self.initial_y - self.current_y
#                 twist_msg = Twist()
#                 twist_msg.linear.x = self.linear_speed
#                 print(self.initial_y, self.current_y, error)

#                 if fabs(error) >= 0.01:  # Tolerance for position control
#                     # Calculate control output
#                     angular_speed = self.Kp_position * error + self.Ki_position * self.error_integral_position + self.Kd_position *1.3 * (error - self.prev_error_position)
#                     self.error_integral_position += error
#                     self.prev_error_position = error
#                     twist_msg.angular.z = -angular_speed
#                 self.vel_pub.publish(twist_msg)
#                 self.rate.sleep()
#         stop_msg = Twist()
#         self.vel_pub.publish(stop_msg)

# if __name__ == '__main__':
#     try:
#         move_rotate_move_obj = MoveRotateMove()
#         move_rotate_move_obj.rotate(target_yaw=pi / 2)
#         move_rotate_move_obj.move_forward(dir="y", distance_to_travel=6.0)
#         move_rotate_move_obj.rotate(target_yaw=pi)
#         move_rotate_move_obj.move_forward(dir="x", distance_to_travel=20.0)
#         move_rotate_move_obj.rotate(target_yaw=-pi / 2)
#         move_rotate_move_obj.move_forward(dir="y", distance_to_travel=6.0, pid_dir = "minus")
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('wall_follower')

        # Subscribe to LaserScan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Publisher for sending velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize state variables
        self.found_wall = False
        self.turning_left = False
        self.moving_forward = True

    def laser_callback(self, msg):
        # Get the minimum distance from the LaserScan data
        min_distance = min(msg.ranges)

        print(f"Laser data received: min_distance = {min_distance:.2f} meters")

        # If moving forward, check for wall detection
        if self.moving_forward:
            if min_distance < 1.0:  # Wall detected
                print("Wall detected! Stopping and turning left.")
                self.moving_forward = False
                self.turn_left()

            # Continue moving forward until a wall is found
            else:
                print("No wall detected, moving forward.")
                self.move_forward()

        # If wall is detected and robot is not moving forward anymore, follow the wall
        if self.found_wall and not self.turning_left:
            self.follow_wall(msg)

    def move_forward(self):
        # Command to move the robot forward
        move_cmd = Twist()
        move_cmd.linear.x = 0.25  # move forward with 0.25 m/s
        self.cmd_vel_pub.publish(move_cmd)

    def turn_left(self):
        # Command to turn the robot to the left
        move_cmd = Twist()
        move_cmd.angular.z = 0.5  # turn left
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)  # turn for 1 second
        self.turning_left = True
        print("Finished turning left. Now following the wall.")
        self.found_wall = True

    def follow_wall(self, msg):
        # Set the following distance (how close you want to stay to the wall)
        desired_distance = 0.5  # meters
        min_distance = min(msg.ranges)

        print(f"Following wall: min_distance = {min_distance:.2f} meters")

        # Create a new velocity command
        move_cmd = Twist()

        # If too close to the wall, move backward
        if min_distance < desired_distance:
            print("Too close to the wall! Moving backward.")
            move_cmd.linear.x = -0.1  # move backward
            move_cmd.angular.z = 0.2  # adjust angle
        # If too far from the wall, move forward and turn slightly
        elif min_distance > desired_distance:
            print("Too far from the wall! Moving forward and turning left.")
            move_cmd.linear.x = 0.2  # move forward
            move_cmd.angular.z = -0.2  # turn left slightly
        else:
            print("At the correct distance from the wall. Moving forward.")
            move_cmd.linear.x = 0.2  # move forward
            move_cmd.angular.z = 0.0  # no turning
        
        # Publish the velocity command to follow the wall
        self.cmd_vel_pub.publish(move_cmd)

    def start(self):
        # Keep the node running
        rospy.spin()


if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        wall_follower.start()
    except rospy.ROSInterruptException:
        pass
