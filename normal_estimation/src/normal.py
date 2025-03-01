#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from normal_estimation.msg import PointsWithNormal
import numpy as np

class LaserProcessor:
    def __init__(self):
        rospy.init_node('laser_processor', anonymous=True)
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.points = []
        self.normals = []

        self.points_with_normals_publisher = rospy.Publisher("/points_with_normals", PointsWithNormal, queue_size=10)

    def callback(self, data):
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Filter out invalid values (inf, nan)
        valid_indices = np.isfinite(x) & np.isfinite(y)
        self.points = np.column_stack((x[valid_indices], y[valid_indices]))
        self.header = data.header

    def compute_normal(self, point, previous_point, next_point, robot_position):
        # Compute direction vectors
        direction_prev = point - previous_point
        direction_next = next_point - point

        direction = 0.5 * (direction_prev + direction_next)

        # Rotate direction vector by 90 degrees
        normal = np.array([direction[1], -direction[0]])

        robot_to_point = point - robot_position
        angle_diff = np.arccos(np.dot(normal, robot_to_point) / (np.linalg.norm(normal) * np.linalg.norm(robot_to_point)))
        if angle_diff < np.pi / 2:  # Angle less than 90 degrees means normal points towards the robot
            normal *= -1  # Reverse direction

        # Normalize the normal vector
        return normal / np.linalg.norm(normal)

    def process_points(self):
        if len(self.points) == 0:
            return

        robot_position = np.array([0, 0])

        self.normals = []


        for i in range((len(self.points) - 10)):

            prev_index = (i - 1) % (len(self.points) - 10)
            next_index = (i + 1) % (len(self.points) - 10)

            if prev_index < 0 or prev_index >= (len(self.points) - 10):
                previous = None
            else:
                previous = self.points[prev_index]
                
            if next_index < 0 or next_index >= (len(self.points) - 10):
                next = None
            else:
                next = self.points[next_index]

            print(f"i: {i}, prev_index: {prev_index}, next_index: {next_index}, len(self.points): {len(self.points)}, len -3: {len(self.points) - 10}")


            previous_point = self.points[prev_index-10]
            next_point = self.points[next_index-10]

            normal = self.compute_normal(self.points[i-10], previous_point, next_point, robot_position)

            self.normals.append(normal)

        self.publish_normals()

    def publish_normals(self):
        points_with_normals_msg = PointsWithNormal()

        points_with_normals_msg.header = self.header

        points_with_normals_msg.points_x = [point[0] for point in self.points]
        points_with_normals_msg.points_y = [point[1] for point in self.points]
        points_with_normals_msg.normals_x = [normal[0] for normal in self.normals]
        points_with_normals_msg.normals_y = [normal[1] for normal in self.normals]

        self.points_with_normals_publisher.publish(points_with_normals_msg)

if __name__ == '__main__':
    laser_processor = LaserProcessor()
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        laser_processor.process_points()
        # rate.sleep()
