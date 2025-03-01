#ifndef ObservationModel_H
#define ObservationModel_H

#include "particle.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;
typedef pcl::octree::OctreePointCloudSearch<PointNormal> OctreeNormal;

struct Correspondence {
    PointNormal scan_pt;
    PointNormal scan_pt_transformed;
    PointNormal map_pt;
    bool reliable_for_yaw_estimation;
};

class ObservationModel {
    public:
        ObservationModel(PointCloudNormal::Ptr map_cloud, int num_observations, float lf_obs_std_dev, float hf_obs_std_dev, float max_nn_dist, float max_nn_normal_ang_diff);
        void setInputCloud(PointCloudNormal::Ptr cloud);
        bool getLikelihoodDistribution(Particle &particle, Eigen::Vector3d &distribution_mean, Eigen::Matrix3d &lf_distribution_precision
                                       , Eigen::Matrix3d &hf_distribution_precision, float& log_likelihood_factor); 
    private:
        int num_observations_;
        float lf_obs_std_dev_;
        float hf_obs_std_dev_;
        float max_nn_sqr_dist_;
        float min_cos_nn_normal_angle_diff_;
        OctreeNormal pos_x_map_octree_;
        OctreeNormal neg_x_map_octree_;
        OctreeNormal pos_y_map_octree_;
        OctreeNormal neg_y_map_octree_;
        PointCloudNormal::Ptr input_cloud_;
        PointCloudNormal::Ptr pos_x_map_cloud_;
        PointCloudNormal::Ptr neg_x_map_cloud_;
        PointCloudNormal::Ptr pos_y_map_cloud_;
        PointCloudNormal::Ptr neg_y_map_cloud_;
        void initializeMapClouds(PointCloudNormal::Ptr map_cloud);
        void computeAndAddCorrespondence(PointNormal& scan_pt, PointNormal& scan_pt_transformed, OctreeNormal& octree_map
                                         , PointCloudNormal::Ptr map_cloud, std::vector<Correspondence>& correspondences); 
        void getCorrespondences(PointCloudNormal::Ptr cloud, PointCloudNormal::Ptr transformed_cloud, std::vector<Correspondence>& correspondences);
};

#endif
