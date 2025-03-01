#include "pcl_ros/point_cloud.h"
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

ros::Publisher map_cloud_pub, map_normals_cloud_pub;
tf::TransformListener *tf_listener_ptr;
tf::TransformBroadcaster *tf_broadcaster_ptr;

PointCloud::Ptr map_cloud(new PointCloud());
PointCloud::Ptr key_frames_cloud(new PointCloud());
PointCloudNormal::Ptr map_normals_cloud(new PointCloudNormal());
pcl::octree::OctreePointCloudSearch<PointT> key_frames_octree(1.0);

pcl::VoxelGrid<PointT> voxel_grid_filter;
pcl::VoxelGrid<PointNormal> voxel_grid_filter_normals;

template<typename T>
void publishCloud(T& cloud, ros::Publisher& publisher, std_msgs::Header header){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = header.frame_id;
    output.header.stamp = header.stamp;
    publisher.publish(output);
}

void poseCb(const nav_msgs::Odometry::ConstPtr& pose_msg) {
    tf::Transform transform;
    tf::poseMsgToTF(pose_msg->pose.pose, transform);
    tf::StampedTransform stamped_transform(transform, pose_msg->header.stamp, "/map", "/base_footprint2");
    tf_broadcaster_ptr->sendTransform(stamped_transform);
}

void cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::PointCloud2ConstPtr& normals_cloud_msg){
    pcl::PCLPointCloud2 pcl_pc2, normals_pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl_conversions::toPCL(*normals_cloud_msg, normals_pcl_pc2);

    PointCloud::Ptr cloud (new PointCloud ());
    PointCloudNormal::Ptr normals_cloud (new PointCloudNormal ());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    pcl::fromPCLPointCloud2(normals_pcl_pc2, *normals_cloud);

    try {
        tf::StampedTransform map_basefootprint_transform, basefootprint_lidar_transform;
        // tf_listener_ptr->lookupTransform("/map", "/base_footprint2", cloud_msg->header.stamp, map_basefootprint_transform);
        tf_listener_ptr->lookupTransform("/map", "/base_footprint2", ros::Time(0), map_basefootprint_transform);
        tf_listener_ptr->lookupTransform("/base_link", "/base_scan", ros::Time(0), basefootprint_lidar_transform); //rslidar
        tf::Transform map_lidar_transform = map_basefootprint_transform * basefootprint_lidar_transform;
    
        tf::Vector3 lidar_pos = map_lidar_transform.getOrigin();
        PointT key_frame(lidar_pos.getX(), lidar_pos.getY(), lidar_pos.getZ());

        std::vector<int> key_frame_idx;
        std::vector<float> key_frame_sqr_dists;
        int num_neighbors = key_frames_octree.radiusSearch(key_frame, 1.0, key_frame_idx, key_frame_sqr_dists); 
        if (num_neighbors == 0){
            key_frames_octree.addPointToCloud(key_frame, key_frames_cloud);
            std::cout << "New key frame added: " << key_frame.x << "," << key_frame.y << "," << key_frame.z << std::endl;

            PointCloud::Ptr transformed_cloud (new PointCloud ());
            pcl_ros::transformPointCloud (*cloud, *transformed_cloud, map_lidar_transform);
            PointCloud::Ptr transformed_cloud_downsampled (new PointCloud ());
            voxel_grid_filter.setInputCloud(transformed_cloud);
            voxel_grid_filter.filter(*transformed_cloud_downsampled);
     
            *map_cloud += *transformed_cloud_downsampled;
            PointCloud::Ptr map_cloud_downsampled (new PointCloud ());
            voxel_grid_filter.setInputCloud(map_cloud);
            voxel_grid_filter.filter(*map_cloud_downsampled);
     
            std_msgs::Header map_cloud_pub_header = cloud_msg->header;  
            map_cloud_pub_header.frame_id = "map";
            publishCloud(map_cloud_downsampled, map_cloud_pub, map_cloud_pub_header); 

            PointCloudNormal::Ptr transformed_normals_cloud (new PointCloudNormal ());
            pcl_ros::transformPointCloudWithNormals (*normals_cloud, *transformed_normals_cloud, map_lidar_transform);
            PointCloudNormal::Ptr transformed_normals_cloud_downsampled (new PointCloudNormal ());
            voxel_grid_filter_normals.setInputCloud(transformed_normals_cloud);
            voxel_grid_filter_normals.filter(*transformed_normals_cloud_downsampled);
            
            
            *map_normals_cloud += *transformed_normals_cloud_downsampled;
            PointCloudNormal::Ptr map_normals_cloud_downsampled (new PointCloudNormal ());
            voxel_grid_filter_normals.setInputCloud(map_normals_cloud);
            voxel_grid_filter_normals.filter(*map_normals_cloud_downsampled);
     
            std_msgs::Header map_normals_cloud_pub_header = cloud_msg->header;
            map_normals_cloud_pub_header.frame_id = "map";
            publishCloud(map_normals_cloud_downsampled, map_normals_cloud_pub, map_normals_cloud_pub_header); 
        }

        // std::cout << "size of normal"<<map_normals_cloud->size()<<std::endl;
        // std::cout << "size of map"<<map_cloud->size()<<std::endl;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    } 
}

int main(int argc, char** argv)
{
    key_frames_octree.setInputCloud(key_frames_cloud);
    voxel_grid_filter.setLeafSize(0.03, 0.03, 0.03);
    voxel_grid_filter_normals.setLeafSize(0.03, 0.03, 0.03);

    ros::init (argc, argv, "sim_mapping_node");
    ros::NodeHandle nh("~");

    tf_listener_ptr = new tf::TransformListener();
    tf_broadcaster_ptr = new tf::TransformBroadcaster();

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/rslidar_points", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> normals_cloud_sub(nh, "/laser_cloud_normals", 10);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(cloud_sub, normals_cloud_sub, 10);
    sync.registerCallback(boost::bind(&cloudCb, _1, _2));

    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, poseCb); //ground_truth_pose
    map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_cloud", 1);
    map_normals_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_normals_cloud", 1);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }



    PointCloudNormal::Ptr map_normals_cloud_downsampled (new PointCloudNormal ());
    voxel_grid_filter_normals.setInputCloud(map_normals_cloud);
    voxel_grid_filter_normals.filter(*map_normals_cloud_downsampled);
    pcl::io::savePCDFileBinary("/tmp/sim_ground_truth_normals_map.pcd", *map_normals_cloud_downsampled);
     
    PointCloud::Ptr map_cloud_downsampled (new PointCloud ());
    voxel_grid_filter.setInputCloud(map_cloud);
    voxel_grid_filter.filter(*map_cloud_downsampled);
    pcl::io::savePCDFileBinary("/tmp/sim_ground_truth_map.pcd", *map_cloud_downsampled);
     
    return 0;
}

