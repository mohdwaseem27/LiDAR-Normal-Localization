#ifndef ApproxNormalEstimation_H_
#define ApproxNormalEstimation_H_

#include <pcl/point_types.h>
#include "pcl/point_cloud.h"
#include <Eigen/Core>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

class ApproxNormalEstimation{
    public:
        ApproxNormalEstimation(int nn_height, int min_nn_width, int max_nn_width, float max_row_nn_dist
                               , float min_nn_major_axes_var, float max_nn_minor_axis_var, float max_pt_to_normal_ang
                               , int row_downsample_factor, int col_downsample_factor);
        void computeSurfaceNormals (const PointCloud::Ptr cloud_in, PointCloudNormal::Ptr cloud_normals); 
    private:
        int nn_half_height_;
        int min_nn_half_width_;
        int max_nn_half_width_;
        float max_row_nn_dist_sqrd_;
        float max_nn_minor_axis_var_;
        float min_nn_major_axes_var_;
        float max_pt_to_normal_ang_;
        float max_valid_nn_normal_dist_;
        int row_downsample_factor_;
        int col_downsample_factor_;
        
        PointNormal createNaNXYZNormalPoint ();
        bool getNearestNeighborCloud(const PointCloud::Ptr cloud_in, int row, int col, PointCloud::Ptr cloud_nn);
        bool getApproxNormal(const PointCloud::Ptr cloud_in, int row, int col, Eigen::Vector3f& normal_vec);
        int getNumNearestNeighborOutliers(PointT pt, PointCloud::Ptr cloud_nn, Eigen::Vector3f eigen_vecs);
};

#endif
