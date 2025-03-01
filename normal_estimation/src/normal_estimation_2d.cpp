#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include "pcl_ros/point_cloud.h"
#include <normal_estimation/normal_estimation.h>
#include <normal_estimation/PointsWithNormal.h>

laser_geometry::LaserProjection projector_;
ros::Publisher cloud_pub;
ros::Publisher scan_cloud_pub;

template<typename T>
void publishCloud(T& cloud, ros::Publisher& publisher, std_msgs::Header header){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = header.frame_id;
    output.header.stamp = header.stamp;
    publisher.publish(output);
}

// void publishNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const std_msgs::Header& header) {
//     // Publish cloud with normals
//     PointCloudNormal::Ptr cloud_with_normals(new PointCloudNormal());
//     std::cout << "\npcl_cloud: " << pcl_cloud->points.size() <<"\n";
//     std::cout << "\ncloud_normals: " << cloud_normals->points.size() <<"\n";
//     pcl::concatenateFields(*pcl_cloud, *cloud_normals, *cloud_with_normals);
//     publishCloud(cloud_with_normals, cloud_pub, header);
//     // std::cout << "\ncloud_with_normals:\n";
//     // for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
//     //     std::cout << "Point " << i << ": "
//     //               << "(" << cloud_with_normals->points[i].x << ", "
//     //               << cloud_with_normals->points[i].y << ", "
//     //               << cloud_with_normals->points[i].z << ") - "
//     //               << "Normal: "
//     //               << "(" << cloud_with_normals->points[i].normal_x << ", "
//     //               << cloud_with_normals->points[i].normal_y << ", "
//     //               << cloud_with_normals->points[i].normal_z << ")\n";
//     // }
// }

void normalsCallback(const normal_estimation::PointsWithNormal::ConstPtr& points_with_normal_msg)  {
    // Clear previous normals
    // cloud_normals->clear();
    // tf::TransformListener listener;
    // tf::StampedTransform transform;

    // try {
    //     // Wait for the transform from /points_with_normals frame to /base_footprint frame
    //     listener.waitForTransform("/base_link", points_with_normal_msg->header.frame_id,
    //                                ros::Time(0), ros::Duration(1.0));
    //     listener.lookupTransform("/base_link", points_with_normal_msg->header.frame_id,
    //                               ros::Time(0), transform);

    // } catch (tf::TransformException ex) {
    //     ROS_ERROR("%s",ex.what());
    //     return;
    // }

    sensor_msgs::PointCloud transformed_cloud;

    // Transform each point in the cloud
    for (size_t i = 0; i < points_with_normal_msg->points_x.size(); ++i) {
        tf::Point pt(points_with_normal_msg->points_x[i], points_with_normal_msg->points_y[i], 0.0); // Z value assumed to be 0
        tf::Point pt_transformed = pt; //transform * pt;
        
        // Add the transformed point to the new cloud
        geometry_msgs::Point32 transformed_point;
        transformed_point.x = pt_transformed.x();
        transformed_point.y = pt_transformed.y();
        transformed_point.z = pt_transformed.z();
        transformed_cloud.points.push_back(transformed_point);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert sensor_msgs::PointCloud to pcl::PointCloud<pcl::PointXYZ>
    for (const auto& point : transformed_cloud.points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;  // Keep the original Z value
        pcl_cloud->push_back(pcl_point);
    }

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Fill in the received normal vectors
    for (size_t i = 0; i < points_with_normal_msg->points_x.size(); ++i) {
        pcl::Normal normal;
        normal.normal_x = points_with_normal_msg->normals_x[i];
        normal.normal_y = points_with_normal_msg->normals_y[i];
        normal.normal_z = 0.0; // Assuming 2D, so setting z component to 0
        cloud_normals->push_back(normal);
    }
    PointCloudNormal::Ptr cloud_with_normals(new PointCloudNormal());
    pcl::concatenateFields(*pcl_cloud, *cloud_normals, *cloud_with_normals);
    publishCloud(cloud_with_normals, cloud_pub, points_with_normal_msg->header);
    std::cout << "\ncloud_with_normals:\n";
    for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
        std::cout << "Point " << i << ": "
                  << "(" << cloud_with_normals->points[i].x << ", "
                  << cloud_with_normals->points[i].y << ", "
                  << cloud_with_normals->points[i].z << ") - "
                  << "Normal: "
                  << "(" << cloud_with_normals->points[i].normal_x << ", "
                  << cloud_with_normals->points[i].normal_y << ", "
                  << cloud_with_normals->points[i].normal_z << ")\n";
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    tf::TransformListener listener_;

            sensor_msgs::PointCloud cloud;
        try {
            ros::Time scan_time = ros::Time::now();
            // Wait for the transform to become available
            // if (!listener_.waitForTransform("base_link", "base_laser", scan_time, ros::Duration(2.0))) {
            //     ROS_WARN_STREAM("Timeout waiting for transform from " << scan_in->header.frame_id << " to /base_link");
            //     return;
            // }
                // projector_.transformLaserScanToPointCloud("/base_link", *scan_in, cloud, listener_);
                projector_.projectLaser(*scan_in, cloud);


            
        } catch (tf::TransformException& ex) {
            ROS_WARN_STREAM("Transform exception: " << ex.what());
        }
    // std::cout << scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size() * scan_in->time_increment) << std::endl;


    // tf::TransformListener listener_;

    // if (!listener_.waitForTransform("/base_link", scan_in->header.frame_id,
    //                                ros::Time(0), ros::Duration(1.0))) {  // Increased timeout duration to 5 seconds
    //     ROS_WARN("Could not get transform from %s to /base_link within 5 seconds", scan_in->header.frame_id.c_str());
    //     return;
    // }

    // std::cout << cloud << std::endl;
    // sensor_msgs::PointCloud cloud;
    // projector_.transformLaserScanToPointCloud("/base_link", *scan_in, cloud, listener_);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert sensor_msgs::PointCloud to pcl::PointCloud<pcl::PointXYZ>
    for (const auto& point : cloud.points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;  // Keep the original Z value
        pcl_cloud->push_back(pcl_point);
    }
    std::cout << *pcl_cloud << std::endl;
    // std::cout << "\npcl_cloud: " << pcl_cloud->points.size() <<"\n";
    publishCloud(pcl_cloud, scan_cloud_pub, scan_in->header);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_estimation_2d");
    ros::NodeHandle nh("~");

    ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scanCallback);
    ros::Subscriber normals_sub = nh.subscribe("/points_with_normals", 1, normalsCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_normals", 1);
    scan_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points", 1); 

    ros::spin();

    return 0;
}