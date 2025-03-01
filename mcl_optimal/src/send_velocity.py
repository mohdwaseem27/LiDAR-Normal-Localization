#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading

class MoveTurtlebot:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('send_velocity', anonymous=True)

        # Publisher to send velocity commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to receive odometry data
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.stop_requested = False

        # Initial position
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_traveled = 0.0

        # Control rate
        self.rate = rospy.Rate(10)  # 10 Hz

        key_thread = threading.Thread(target=self.wait_for_key_press)
        key_thread.daemon = True  # Ensure the thread ends when the main program exits
        key_thread.start()

    def odom_callback(self, msg):
        # Update current position based on odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def wait_for_key_press(self):
        # Wait for 's' key press to stop the robot
        input("Press 's' and hit Enter to stop the robot\n")
        self.stop_requested = True
    
    def rotate_in_circle(self, linear_speed, angular_speed):
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        start_time = rospy.Time.now()
        duration = abs(2 * math.pi / angular_speed)

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.stop_requested:  # Check if stop was requested
                rospy.loginfo("Stopping due to user input")
                break
            self.pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

    def move_forward(self, linear_speed, distance):
        # Get the starting position
        self.start_x = self.current_x
        self.start_y = self.current_y

        # Create a Twist message for linear movement
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = 0.0

        # Start moving the robot
        while self.distance_traveled < distance:
            self.pub.publish(vel_msg)  # Publish the velocity command
            self.distance_traveled = self.calculate_distance()  # Update the traveled distance
            self.rate.sleep()  # Maintain the loop rate

        # Stop the robot after reaching the desired distance
        vel_msg.linear.x = 0
        self.pub.publish(vel_msg)  # Publish the stop command

    def calculate_distance(self):
        # Calculate the Euclidean distance from the starting position
        return math.sqrt((self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2)

if __name__ == '__main__':
    try:
        mover = MoveTurtlebot()
        # mover.move_forward(0.22, 5.2)  # Move forward at 0.22 m/s for 5 meters
        mover.rotate_in_circle(0.25, -0.5)
    except rospy.ROSInterruptException:
        pass
