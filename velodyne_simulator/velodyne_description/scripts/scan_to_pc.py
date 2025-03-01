import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32, create_cloud
import numpy as np

def scan_callback(scan_msg):
    # Step 2: Convert the 2D LaserScan to organized 3D point cloud
    angle_min = scan_msg.angle_min
    angle_increment = scan_msg.angle_increment
    range_max = scan_msg.range_max
    num_ranges = len(scan_msg.ranges)

    # Create an empty grid with default values set to NaN (unmeasured points)
    grid_size_x = 200  # Number of grid cells along the x-axis (adjust this based on your environment)
    grid_size_y = 200  # Number of grid cells along the y-axis (adjust this based on your environment)
    grid = np.full((grid_size_x, grid_size_y), np.nan, dtype=np.float32)

    for i, range_measurement in enumerate(scan_msg.ranges):
        if range_measurement < range_max:
            # Calculate the Cartesian coordinates (x, y) of the point
            angle = angle_min + i * angle_increment
            x = range_measurement * np.cos(angle)
            y = range_measurement * np.sin(angle)

            # Convert Cartesian coordinates to grid indices
            grid_x = int((x - min_x) / resolution)
            grid_y = int((y - min_y) / resolution)

            # Set the value in the grid to the measured range
            if 0 <= grid_x < grid_size_x and 0 <= grid_y < grid_size_y:
                grid[grid_x, grid_y] = range_measurement

    # Step 3: Publish the organized point cloud
    header = scan_msg.header
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    point_cloud_msg = create_cloud(header, fields, grid.reshape(-1, 1, order='F'))
    point_cloud_publisher.publish(point_cloud_msg)

if __name__ == '__main__':
    rospy.init_node('scan_to_organized_pointcloud_node')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    point_cloud_publisher = rospy.Publisher('/rslidar_points', PointCloud2, queue_size=1)

    # Define the grid parameters
    resolution = 0.1  # Define the grid resolution (adjust this based on your requirements)
    grid_size_x = 200  # Number of grid cells along the x-axis (adjust this based on your environment)
    grid_size_y = 200  # Number of grid cells along the y-axis (adjust this based on your environment)
    min_x = -10.0  # Minimum x-coordinate of the grid
    min_y = -10.0  # Minimum y-coordinate of the grid

    rospy.spin()
