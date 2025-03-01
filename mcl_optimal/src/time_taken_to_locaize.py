#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import csv
import os
from tf.transformations import euler_from_quaternion

num_particles = 1  # Default value
run = 5  # Default value
algorithm = "Optimal"  # Default algorithm (can also be 'optimal')


class PoseComparator:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.latest_odom = None
        self.latest_pose = None
        self.rmse_values = []
        self.start_time = None
        self.time_taken = None
        self.csv_saved = False  # Flag to track if CSV has been saved
        self.run_data = {}

    def odom_callback(self, odom_msg):
        self.latest_odom = odom_msg.pose.pose
        self.calculate_average_distance()

    def pose_callback(self, pose_msg):
        self.latest_pose = pose_msg.pose.pose
        self.calculate_average_distance()

    def calculate_average_distance(self):
        if self.latest_odom is None or self.latest_pose is None:
            return

        # Calculate RMSE for position
        squared_error = (self.latest_odom.position.x - self.latest_pose.position.x) ** 2 + \
                        (self.latest_odom.position.y - self.latest_pose.position.y) ** 2 + \
                        (self.latest_odom.position.z - self.latest_pose.position.z) ** 2

        rmse_position = math.sqrt(squared_error)

        # Convert quaternion to roll, pitch, and yaw
        odom_orientation = [
            self.latest_odom.orientation.x,
            self.latest_odom.orientation.y,
            self.latest_odom.orientation.z,
            self.latest_odom.orientation.w
        ]
        pose_orientation = [
            self.latest_pose.orientation.x,
            self.latest_pose.orientation.y,
            self.latest_pose.orientation.z,
            self.latest_pose.orientation.w
        ]
        _, _, odom_yaw = euler_from_quaternion(odom_orientation)
        _, _, pose_yaw = euler_from_quaternion(pose_orientation)

        # Calculate RMSE for yaw (normalize angle difference)
        yaw_error = odom_yaw - pose_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize angle to [-pi, pi]
        rmse_yaw = abs(yaw_error)
        # print(rmse_position, rmse_position)
        if self.start_time is None:  # Start the timer only once
            self.start_time = rospy.get_time()  # Record the current time

        # Check if RMSE for position and yaw are within thresholds
        if rmse_position < 0.15 and rmse_yaw < 0.15: 
            # print(rmse_position, rmse_yaw)

            # Calculate the time taken to localize
            self.time_taken = rospy.get_time() - self.start_time
            # If num_particles is not in run_data, initialize it
            if num_particles not in self.run_data:
                self.run_data[num_particles] = []

            self.run_data[num_particles].append(self.time_taken)

            # Save to CSV only once
            if not self.csv_saved:
                rospy.loginfo(f"Time taken to localize: {self.time_taken} seconds")
                self.save_time_to_csv()  # Save the data
                self.csv_saved = True

    def save_time_to_csv(self):
        # Set the filename for the CSV
        filename = "/home/waseem/Documents/HBRS/RnD/catkin_ws/src/experiments/new/random_wrong_inital_pose/time_taken_to_localize.csv"
        file_exists = os.path.exists(filename)

        # Open the file to append data
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = ['Particles', 'AMCL_run1', 'AMCL_run2', 'AMCL_run3', 'AMCL_run4', 'AMCL_run5', 
                        'Optimal_run1', 'Optimal_run2', 'Optimal_run3', 'Optimal_run4', 'Optimal_run5']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write header only if the file does not exist
            if not file_exists:
                writer.writeheader()

            # Write the time taken for each run for the specific algorithm
            for num_particles, times in self.run_data.items():
                row = {'Particles': num_particles}
                
                # Fill in the data for AMCL or Optimal runs
                if algorithm == "AMCL":
                    row[f'AMCL_run{run}'] = times[0] if len(times) > 0 else ''  # Store in the column corresponding to 'run'
                elif algorithm == "Optimal":
                    row[f'Optimal_run{run}'] = times[0] if len(times) > 0 else ''  # Store in the column corresponding to 'run'

                
                # Write the row to the CSV file
                writer.writerow(row)

        rospy.loginfo(f"Time taken to localize for {num_particles} particles: {self.time_taken} seconds")


def main():
    global num_particles, run, algorithm
    rospy.init_node('time_taken_localize')
    pose_comparator = PoseComparator()

    # Let ROS handle the processing of incoming messages
    rospy.spin()  # This will keep the node running and process callbacks

if __name__ == '__main__':
    main()
