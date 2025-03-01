#include "observation_model.h"
#include <pcl/common/transforms.h>
#include <Eigen/SVD>
#include <cmath>
#include <algorithm>

ObservationModel::ObservationModel(PointCloudNormal::Ptr map_cloud, int num_observations, float lf_obs_std_dev, float hf_obs_std_dev
                                   , float max_nn_dist, float max_nn_normal_ang_diff)
                                   : pos_x_map_octree_(0.1), neg_x_map_octree_(0.1), pos_y_map_octree_(0.1), neg_y_map_octree_(0.1) {
    num_observations_ = num_observations;
    lf_obs_std_dev_ = lf_obs_std_dev;
    hf_obs_std_dev_ = hf_obs_std_dev;
    max_nn_sqr_dist_ = pow(max_nn_dist,2);
    min_cos_nn_normal_angle_diff_ = cos(max_nn_normal_ang_diff);
    initializeMapClouds(map_cloud); 
}

void ObservationModel::setInputCloud(PointCloudNormal::Ptr cloud) {
    input_cloud_.reset(new PointCloudNormal ());
    for (size_t i=0; i < cloud->points.size(); i++) {
        PointNormal pt = cloud->points[i];
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.normal_x) 
            && std::isfinite(pt.normal_y)) {
            input_cloud_->points.push_back(cloud->points[i]);
        }
    }
    if (input_cloud_->points.size() > num_observations_) {
        std::random_shuffle(input_cloud_->points.begin(), input_cloud_->points.end());
        input_cloud_->points.resize(num_observations_);
    } else {
        std::cout << "Insufficient number of scan points" << std::endl;
    }
    *cloud = *input_cloud_;
}

bool ObservationModel::getLikelihoodDistribution(Particle &particle, Eigen::Vector3d &distribution_mean, Eigen::Matrix3d &lf_distribution_precision
                                                 , Eigen::Matrix3d &hf_distribution_precision, float& log_likelihood_factor) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity() * particle.getQuat();
    transform.translation() =  particle.getPos();
    PointCloudNormal::Ptr transformed_cloud (new PointCloudNormal ());
    pcl::transformPointCloudWithNormals (*input_cloud_, *transformed_cloud, transform);

    std::vector<Correspondence> correspondences;
    getCorrespondences(input_cloud_, transformed_cloud, correspondences);

    if (correspondences.size() > 1){
        Eigen::Matrix2f robot_rot_2d = particle.getQuat().toRotationMatrix().block(0,0,2,2); 

        Eigen::MatrixXf A(correspondences.size(), 3);
        Eigen::VectorXf b(correspondences.size());
        for (size_t i = 0; i < correspondences.size(); i++){
            PointNormal& scan_pt = correspondences[i].scan_pt;
            PointNormal& scan_pt_transformed = correspondences[i].scan_pt_transformed;
            PointNormal& map_pt = correspondences[i].map_pt;
            Eigen::RowVector2f normal_vec_2d_transposed(map_pt.normal_x, map_pt.normal_y);
            Eigen::RowVector2f nR_2d = normal_vec_2d_transposed * robot_rot_2d;
            Eigen::Vector2f S_scan_pt_2d(-scan_pt.y, scan_pt.x);
            //A.row(i) << nR_2d, nR_2d*S_scan_pt_2d;
            if (correspondences[i].reliable_for_yaw_estimation){
                A.row(i) << nR_2d, nR_2d*S_scan_pt_2d;
            } else { 
                A.row(i) << nR_2d, 0;
            } 
            b(i) = map_pt.curvature - (scan_pt_transformed.x*map_pt.normal_x + scan_pt_transformed.y*map_pt.normal_y + scan_pt_transformed.z*map_pt.normal_z);
        }
        distribution_mean = A.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(b).cast<double>();

        Eigen::Matrix3f H = A.transpose() * A;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver;
        eigen_solver.computeDirect(H.cast<double>(), Eigen::ComputeEigenvectors);
        Eigen::Vector3d eigen_vals = eigen_solver.eigenvalues();
        Eigen::Matrix3d eigen_vecs = eigen_solver.eigenvectors();
        Eigen::Vector3d lf_eigen_vals_adjusted, hf_eigen_vals_adjusted;
        for (size_t i = 0; i < 3; i++){
            // 0.15 implies 0.15rad (9deg) noise in estimated surface normals
            hf_eigen_vals_adjusted(i) = (eigen_vals(i) < 0.15 * 0.15 * correspondences.size()) ? 0.01 : eigen_vals(i)/(hf_obs_std_dev_*hf_obs_std_dev_); 
        }
        hf_distribution_precision = eigen_vecs * hf_eigen_vals_adjusted.asDiagonal() * eigen_vecs.transpose();
        
        for (size_t i = 0; i < 3; i++){
            // 0.15 implies 0.15rad (9deg) noise in estimated surface normals
            lf_eigen_vals_adjusted(i) = (eigen_vals(i) < 0.15 * 0.15 * correspondences.size()) ? 0.01 : eigen_vals(i)/(lf_obs_std_dev_*lf_obs_std_dev_); 
        }
        lf_distribution_precision = eigen_vecs * lf_eigen_vals_adjusted.asDiagonal() * eigen_vecs.transpose();
        int num_observations = input_cloud_->points.size();
        float orig_sqr_dist_sum = (b.transpose() * b) + (num_observations - correspondences.size()) * max_nn_sqr_dist_; 
        float orig_log_likelihood = num_observations * log(1/(lf_obs_std_dev_*sqrt(2*M_PI))) - 0.5 * orig_sqr_dist_sum / pow(lf_obs_std_dev_,2);
        float precision_product = lf_eigen_vals_adjusted(0) * lf_eigen_vals_adjusted(1) * lf_eigen_vals_adjusted(2); 
        float new_log_likelihood = log(sqrt(precision_product/(8*pow(M_PI,3)))) - 0.5 * distribution_mean.transpose() * lf_distribution_precision * distribution_mean;
        log_likelihood_factor = orig_log_likelihood - new_log_likelihood;
        return true;
    } else {
        std::cout << "Zero correspondences found for particle" << std::endl;
        distribution_mean = Eigen::Vector3d(0, 0, 0);

        hf_distribution_precision = Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal();
        Eigen::Vector3d lf_eigen_vals_adjusted(0.01, 0.01, 0.01); 
        lf_distribution_precision = lf_eigen_vals_adjusted.asDiagonal();
        
        int num_observations = input_cloud_->points.size();
        float orig_sqr_dist_sum = num_observations * max_nn_sqr_dist_; 
        float orig_log_likelihood = num_observations * log(1/(lf_obs_std_dev_*sqrt(2*M_PI))) - 0.5 * orig_sqr_dist_sum / pow(lf_obs_std_dev_,2);
        float precision_product = lf_eigen_vals_adjusted(0) * lf_eigen_vals_adjusted(1) * lf_eigen_vals_adjusted(2); 
        float new_log_likelihood = log(sqrt(precision_product/(8*pow(M_PI,3))));
        log_likelihood_factor = orig_log_likelihood - new_log_likelihood;
        return false;
    }
}

void ObservationModel::initializeMapClouds(PointCloudNormal::Ptr map_cloud) {
    pos_x_map_cloud_.reset(new PointCloudNormal ());
    neg_x_map_cloud_.reset(new PointCloudNormal ());
    pos_y_map_cloud_.reset(new PointCloudNormal ());
    neg_y_map_cloud_.reset(new PointCloudNormal ());

    for (size_t i = 0; i < map_cloud->points.size(); i++){
        PointNormal pt = map_cloud->points[i];
        //if (fabs(pt.normal_z) < 0.1){                   // <6deg tilt/pitch
        if (std::isfinite(pt.normal_z)) {
            pt.curvature = pt.x*pt.normal_x + pt.y*pt.normal_y; // + pt.z*pt.normal_z;
            pos_x_map_cloud_->points.push_back(pt);
        }
    }
    std::cout << "pos_x_map_cloud_: " << pos_x_map_cloud_->points.size() << std::endl;
    pos_x_map_octree_.setInputCloud(pos_x_map_cloud_);
    pos_x_map_octree_.addPointsFromInputCloud();
}

void ObservationModel::computeAndAddCorrespondence(PointNormal& scan_pt, PointNormal& scan_pt_transformed, OctreeNormal& octree_map
                                                   , PointCloudNormal::Ptr map_cloud, std::vector<Correspondence>& correspondences) {  
    int nn_pt_id; 
    float nn_sqr_dist;
    octree_map.approxNearestSearch(scan_pt_transformed, nn_pt_id, nn_sqr_dist);
    if (nn_sqr_dist < max_nn_sqr_dist_){
        PointNormal map_pt = map_cloud->points[nn_pt_id];
         float cos_normal_ang = scan_pt_transformed.normal_x*map_pt.normal_x + scan_pt_transformed.normal_y*map_pt.normal_y;// + scan_pt_transformed.normal_z*map_pt.normal_z;
        if (cos_normal_ang > min_cos_nn_normal_angle_diff_) { 
            float scan_pt_dist = sqrt(scan_pt.x * scan_pt.x + scan_pt.y * scan_pt.y); 
            bool reliable_for_yaw_estimation = fabs((scan_pt.x * scan_pt.normal_x + scan_pt.y * scan_pt.normal_y)/scan_pt_dist) < 1.398; // If ang > 0.2rad
            correspondences.push_back(Correspondence{scan_pt, scan_pt_transformed, map_pt, reliable_for_yaw_estimation}); 
        }
    }
}

void ObservationModel::getCorrespondences(PointCloudNormal::Ptr cloud, PointCloudNormal::Ptr transformed_cloud, std::vector<Correspondence>& correspondences) { 
    for (size_t i = 0; i < transformed_cloud->points.size(); i++){
        PointNormal scan_pt = cloud->points[i];
        PointNormal scan_pt_transformed = transformed_cloud->points[i];
        if (std::isfinite(scan_pt_transformed.x) && std::isfinite(scan_pt_transformed.normal_z)){
            computeAndAddCorrespondence(scan_pt, scan_pt_transformed, pos_x_map_octree_, pos_x_map_cloud_, correspondences);
        }
    }
}
