#include "pcl_ros/point_cloud.h"
#include <normal_estimation/approx_normal_estimation.h>

ApproxNormalEstimation *ne;
ros::Publisher cloud_pub;

template<typename T>
void publishCloud(T& cloud, ros::Publisher& publisher, std_msgs::Header header){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = header.frame_id;
    output.header.stamp = header.stamp;
    publisher.publish(output);
}

void reorderPointcloud (PointCloud::Ptr cloud_in){
    PointCloud::Ptr cloud_temp (new PointCloud ());
    *cloud_temp = *cloud_in;
    for( size_t row = 15, row_new = 8; row > 7; row--, row_new++){
        for( size_t col = 0; col < cloud_in->width; col++){
            cloud_in->at(col,row_new) = cloud_temp->at(col,row);
        }
    }
}

void cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    PointCloud::Ptr cloud_in (new PointCloud ());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_in);

    reorderPointcloud(cloud_in);

    PointCloudNormal::Ptr cloud_normals (new PointCloudNormal ());
    ne->computeSurfaceNormals(cloud_in, cloud_normals);
    
    cloud_normals->is_dense = true;
    cloud_normals->height = 1;
    cloud_normals->width = cloud_normals->points.size();
    publishCloud(cloud_normals, cloud_pub, cloud_msg->header);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "normal_estimation");
    ros::NodeHandle nh("~");
      
    float max_row_nn_dist, max_nn_minor_axis_var, min_nn_major_axes_var, max_pt_to_normal_ang;
    int row_downsample_factor, col_downsample_factor;
    nh.param("max_row_nn_dist", max_row_nn_dist, 7e-2f);
    nh.param("max_nn_minor_axis_var", max_nn_minor_axis_var, 2.5e-5f);
    nh.param("min_nn_major_axes_var", min_nn_major_axes_var, 9e-4f);
    nh.param("max_pt_to_normal_ang", max_pt_to_normal_ang, 1.05f);
    nh.param("row_downsample_factor", row_downsample_factor, 2);
    nh.param("col_downsample_factor", col_downsample_factor, 5);
 
    //std::cout << "max_row_nn_dist: " << max_row_nn_dist << std::endl;
    //std::cout << "max_nn_minor_axis_var: " << max_nn_minor_axis_var << std::endl;
    //std::cout << "min_nn_major_axes_var: " << min_nn_major_axes_var << std::endl;
    //std::cout << "max_pt_to_normal_ang: " << max_pt_to_normal_ang << std::endl;
 
    ne = new ApproxNormalEstimation(3, 9, 41, max_row_nn_dist, min_nn_major_axes_var, max_nn_minor_axis_var, 
                                    max_pt_to_normal_ang, row_downsample_factor, col_downsample_factor);  
 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar_points", 1, cloudCb);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar_point_normals", 1);
 
    ros::spin ();
}
