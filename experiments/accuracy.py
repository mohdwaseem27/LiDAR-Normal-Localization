#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import csv
import sys
import tty
import termios

class PoseComparator:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.latest_odom = None
        self.latest_pose = None
        self.record_data = False
        self.rmse_values = []

    def odom_callback(self, odom_msg):
        self.latest_odom = odom_msg.pose.pose

        self.calculate_average_distance()

    def pose_callback(self, pose_msg):
        self.latest_pose = pose_msg.pose.pose
        self.calculate_average_distance()

    def calculate_average_distance(self):
        if self.latest_odom is None or self.latest_pose is None:
            return

        squared_error = (self.latest_odom.position.x - self.latest_pose.position.x)**2 + \
                        (self.latest_odom.position.y - self.latest_pose.position.y)**2 + \
                        (self.latest_odom.position.z - self.latest_pose.position.z)**2

        rmse = math.sqrt(squared_error)

        # rospy.loginfo("Root Mean Square Error (RMSE): {}".format(rmse))

        # Record RMSE values if recording is enabled
        if self.record_data:
            self.rmse_values.append(rmse)

    def start_recording(self):
        rospy.loginfo("Recording started. Press 's' to stop.")

        while not rospy.is_shutdown():
            key = get_key()

            if key == 's':
                rospy.loginfo("Recording stopped.")
                break

    def save_to_csv(self, filename='rmse_values.csv'):
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['RMSE']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for rmse in self.rmse_values:
                writer.writerow({'RMSE': rmse})

        rospy.loginfo("RMSE values saved to {}".format(filename))

def get_key():
    # Function to get a single character from the terminal without waiting for Enter
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rospy.init_node('pose_comparator')
    pose_comparator = PoseComparator()

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == 'w' or key == 'x':
                pose_comparator.record_data = True
                pose_comparator.start_recording()
                pose_comparator.save_to_csv()
                pose_comparator.record_data = False
            elif key == 's':
                pose_comparator.record_data = False
        
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
