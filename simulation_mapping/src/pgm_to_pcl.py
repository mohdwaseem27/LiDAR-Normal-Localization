# import rospy
# import numpy as np
# from nav_msgs.msg import OccupancyGrid

# def save_pcd_with_normals(points_with_normals, file_name):
#     """Save the points with normals and curvature to a PCD file."""
#     num_points = points_with_normals.shape[0]
#     with open(file_name, 'w') as file:
#         file.write('# .PCD v0.7 - Point Cloud Data file format\n')
#         file.write('VERSION 0.7\n')
#         file.write('FIELDS x y z normal_x normal_y normal_z curvature\n')  # Added curvature
#         file.write('SIZE 4 4 4 4 4 4 4\n')  # Added size for curvature
#         file.write('TYPE F F F F F F F\n')  # Added type for curvature
#         file.write('COUNT 1 1 1 1 1 1 1\n')  # Added count for curvature
#         file.write(f'WIDTH {num_points}\n')
#         file.write('HEIGHT 1\n')
#         file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
#         file.write(f'POINTS {num_points}\n')
#         file.write('DATA ascii\n')
#         for point in points_with_normals:
#             file.write(f'{point[0]} {point[1]} {point[2]} {point[3]} {point[4]} {point[5]} {point[6]}\n')  # Added curvature
    
#     rospy.loginfo(f"Point cloud saved to {file_name}")

# def map_callback(data):
#     width = data.info.width
#     height = data.info.height
#     resolution = data.info.resolution
#     origin_x = data.info.origin.position.x
#     origin_y = data.info.origin.position.y

#     occupancy_grid = np.array(data.data).reshape((height, width))

#     points = []
#     normals = []
#     curvatures = []

#     for y in range(height):
#         for x in range(width):
#             if occupancy_grid[y, x] == 100:  # Occupied cell
#                 # Convert map coordinates to real-world coordinates
#                 real_x = x * resolution + origin_x
#                 real_y = y * resolution + origin_y
#                 real_z = 0.0  # Keep z constant

#                 points.append((real_x, real_y, real_z))  # (x, y, z)

#                 # Calculate normals (using neighboring cells)
#                 dx = occupancy_grid[y, x + 1] - occupancy_grid[y, x] if x < width - 1 else 0
#                 dy = occupancy_grid[y + 1, x] - occupancy_grid[y, x] if y < height - 1 else 0

#                 # Normalize the normal vector
#                 mag = np.sqrt(dx ** 2 + dy ** 2)
#                 normal_x = dx / mag if mag > 0 else 0.0
#                 normal_y = dy / mag if mag > 0 else 0.0
#                 normal_z = 0.0  # z-component of the normal

#                 normals.append((normal_x, normal_y, normal_z))  # (normal_x, normal_y, normal_z)

#                 # Calculate curvature
#                 curvature = np.sqrt(dx**2 + dy**2)  # Simplified curvature calculation
#                 curvatures.append(curvature)

#     # Convert lists to numpy arrays
#     points_np = np.array(points, dtype=np.float32)
#     normals_np = np.array(normals, dtype=np.float32)
#     curvatures_np = np.array(curvatures, dtype=np.float32)

#     # Create a structured array to hold points, normals, and curvature
#     structured_points = np.zeros((len(points), 7), dtype=np.float32)  # Changed size to 7
#     structured_points[:, 0:3] = points_np  # x, y, z
#     structured_points[:, 3:6] = normals_np  # normal_x, normal_y, normal_z
#     structured_points[:, 6] = curvatures_np  # curvature

#     # Calculate the centroid
#     centroid = np.mean(points_np, axis=0)

#     # Adjust normals to point inward
#     for i in range(len(points)):
#         vector_to_centroid = centroid - points_np[i]
#         if np.dot(vector_to_centroid, normals_np[i]) < 0:
#             normals_np[i] = -normals_np[i]  # Invert normal if it points outward

#     structured_points[:, 3:6] = normals_np  # Update normals in the structured points

#     # Save the point cloud with normals and curvature to a PCD file
#     save_pcd_with_normals(structured_points, '/home/waseem/Documents/HBRS/RnD/catkin_ws/src/mcl_optimal/maps/map_output.pcd')

# if __name__ == '__main__':
#     rospy.init_node('map_subscriber', anonymous=True)
#     rospy.Subscriber('/map', OccupancyGrid, map_callback)
#     rospy.spin()


import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import os


robot_position = None
radius = 3.0

# def save_pcd_with_normals(points_with_normals, file_name):
#     """Save the points with normals and curvature to a PCD file."""
#     num_points = points_with_normals.shape[0]
#     with open(file_name, 'w') as file:
#         file.write('# .PCD v0.7 - Point Cloud Data file format\n')
#         file.write('VERSION 0.7\n')
#         file.write('FIELDS x y z normal_x normal_y normal_z curvature\n')  # Added curvature
#         file.write('SIZE 4 4 4 4 4 4 4\n')  # Added size for curvature
#         file.write('TYPE F F F F F F F\n')  # Added type for curvature
#         file.write('COUNT 1 1 1 1 1 1 1\n')  # Added count for curvature
#         file.write(f'WIDTH {num_points}\n')
#         file.write('HEIGHT 1\n')
#         file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
#         file.write(f'POINTS {num_points}\n')
#         file.write('DATA ascii\n')
#         for point in points_with_normals:
#             file.write(f'{point[0]} {point[1]} {point[2]} {point[3]} {point[4]} {point[5]} {point[6]}\n')  # Added curvature
    
#     rospy.loginfo(f"Point cloud saved to {file_name}")


def save_pcd_with_normals(points_with_normals, file_name):
    """Save or append points with normals and curvature to a PCD file."""
    
    if os.path.exists(file_name):
        with open(file_name, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.startswith('POINTS'):
                    initial_point_count = int(line.split()[1])
                    break
            else:
                initial_point_count = 0  
        mode = 'a'
    else:
        initial_point_count = 0
        mode = 'w'

    with open(file_name, mode) as file:
        if mode == 'w':  
            file.write('# .PCD v0.7 - Point Cloud Data file format\n')
            file.write('VERSION 0.7\n')
            file.write('FIELDS x y z normal_x normal_y normal_z curvature\n')
            file.write('SIZE 4 4 4 4 4 4 4\n')
            file.write('TYPE F F F F F F F\n')
            file.write('COUNT 1 1 1 1 1 1 1\n')
            file.write(f'WIDTH {initial_point_count + points_with_normals.shape[0]}\n')
            file.write('HEIGHT 1\n')
            file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            file.write(f'POINTS {initial_point_count + points_with_normals.shape[0]}\n')
            file.write('DATA ascii\n')
        else:
            lines[6] = f'WIDTH {initial_point_count + points_with_normals.shape[0]}\n'
            lines[9] = f'POINTS {initial_point_count + points_with_normals.shape[0]}\n'
            with open(file_name, 'w') as f:
                f.writelines(lines)  

        for point in points_with_normals:
            file.write(f'{point[0]} {point[1]} {point[2]} {point[3]} {point[4]} {point[5]} {point[6]}\n')
    
    rospy.loginfo(f"Point cloud {'appended to' if mode == 'a' else 'saved to'} {file_name}")



def robot_position_callback(data):
    """Callback to update robot's position."""
    global robot_position
    robot_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, 0.0])

def map_callback(data):
    global robot_position
    if robot_position is None:
        rospy.logwarn("Waiting for robot position...")
        return

    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y

    occupancy_grid = np.array(data.data).reshape((height, width))

    points = []
    normals = []
    curvatures = []

    for y in range(height):
        for x in range(width):
            if occupancy_grid[y, x] == 100:  
                real_x = x * resolution + origin_x
                real_y = y * resolution + origin_y
                real_z = 0.0 
                distance_to_robot = np.sqrt((real_x - robot_position[0])**2 + (real_y - robot_position[1])**2)
                if distance_to_robot > radius:
                    continue  # Skip points outside the specified radius


                points.append((real_x, real_y, real_z))  

                dx = occupancy_grid[y, x + 1] - occupancy_grid[y, x] if x < width - 1 else 0
                dy = occupancy_grid[y + 1, x] - occupancy_grid[y, x] if y < height - 1 else 0

                mag = np.sqrt(dx ** 2 + dy ** 2)
                normal_x = dx / mag if mag > 0 else 0.0
                normal_y = dy / mag if mag > 0 else 0.0
                normal_z = 0.0  

                normals.append((normal_x, normal_y, normal_z))  

                curvature = np.sqrt(dx**2 + dy**2)  
                curvatures.append(curvature)

    points_np = np.array(points, dtype=np.float32)
    normals_np = np.array(normals, dtype=np.float32)
    curvatures_np = np.array(curvatures, dtype=np.float32)

    structured_points = np.zeros((len(points), 7), dtype=np.float32)  
    structured_points[:, 0:3] = points_np  
    structured_points[:, 3:6] = normals_np  
    structured_points[:, 6] = curvatures_np  

    for i in range(len(points)):
        vector_to_robot = robot_position - points_np[i]
        if np.dot(vector_to_robot, normals_np[i]) < 0:
            normals_np[i] = -normals_np[i]  # Invert normal if it points outward

    structured_points[:, 3:6] = normals_np  

    save_pcd_with_normals(structured_points, '/home/waseem/Documents/HBRS/RnD/catkin_ws/src/mcl_optimal/maps/map_output.pcd')

if __name__ == '__main__':
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/odom', Odometry, robot_position_callback)  # Subscribe to robot's pose topic
    rospy.spin()
