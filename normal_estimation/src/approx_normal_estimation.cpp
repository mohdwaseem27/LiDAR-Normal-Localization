#include <cmath>
#include <Eigen/Eigenvalues>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <normal_estimation/approx_normal_estimation.h>

ApproxNormalEstimation::ApproxNormalEstimation(int nn_height, int min_nn_width, int max_nn_width, float max_row_nn_dist
                                   , float min_nn_major_axes_var, float max_nn_minor_axis_var, float max_pt_to_normal_ang
                                   , int row_downsample_factor, int col_downsample_factor){
    nn_half_height_ = std::floor(nn_height/2);
    min_nn_half_width_ = std::floor((min_nn_width-1)/2);
    max_nn_half_width_ = std::floor((max_nn_width-1)/2);
    max_row_nn_dist_sqrd_ = pow(max_row_nn_dist,2);
    max_nn_minor_axis_var_ = max_nn_minor_axis_var;
    min_nn_major_axes_var_ = min_nn_major_axes_var;
    max_pt_to_normal_ang_ = max_pt_to_normal_ang;
    max_valid_nn_normal_dist_ = 2*sqrt(max_nn_minor_axis_var);
    row_downsample_factor_ = row_downsample_factor; 
    col_downsample_factor_ = col_downsample_factor; 
}

PointNormal ApproxNormalEstimation::createNaNXYZNormalPoint (){
    PointNormal pt_normal;
    pt_normal.x = std::numeric_limits<double>::quiet_NaN();
    pt_normal.y = std::numeric_limits<double>::quiet_NaN();
    pt_normal.z = std::numeric_limits<double>::quiet_NaN();
    return pt_normal;
}

bool ApproxNormalEstimation::getApproxNormal(const PointCloud::Ptr cloud_in, int row, int col, Eigen::Vector3f& normal_vec){
    if (row - nn_half_height_ < 0 || row + nn_half_height_ >= cloud_in->height){ return false; }
    if (col - max_nn_half_width_ < 0 || col + max_nn_half_width_ >= cloud_in->width){ return false; }
    PointT pt_left = cloud_in->at(col-min_nn_half_width_, row); 
    PointT pt_right = cloud_in->at(col+min_nn_half_width_, row); 
    PointT pt_upper = cloud_in->at(col, row+nn_half_height_); 
    PointT pt_lower = cloud_in->at(col, row-nn_half_height_); 
    if (!std::isfinite(pt_left.x) || !std::isfinite(pt_right.x) || !std::isfinite(pt_upper.x) || !std::isfinite(pt_lower.x)){ return false; }
    Eigen::Vector3f horizontal_vec(pt_right.x-pt_left.x, pt_right.y-pt_left.y, pt_right.z-pt_left.z);
    Eigen::Vector3f vertical_vec(pt_upper.x-pt_lower.x, pt_upper.y-pt_lower.y, pt_upper.z-pt_lower.z);
    normal_vec = horizontal_vec.cross(vertical_vec) / (horizontal_vec.norm()*vertical_vec.norm());
}

bool ApproxNormalEstimation::getNearestNeighborCloud(const PointCloud::Ptr cloud_in, int row, int col, PointCloud::Ptr cloud_nn){
    if (row - nn_half_height_ < 0 || row + nn_half_height_ >= cloud_in->height){ return false; }
    if (col - max_nn_half_width_ < 0 || col + max_nn_half_width_ >= cloud_in->width){ return false; }
    int cloud_nn_idx = 0;
    cloud_nn->points.resize((2*nn_half_height_+1)*(2*max_nn_half_width_ + 1));
    for( int row_shift = -nn_half_height_; row_shift <= nn_half_height_; row_shift++){
        PointT pt_row_mid = cloud_in->at(col, row+row_shift);
        if (!std::isfinite(pt_row_mid.x)){ return false; }
        int num_pts_row = 0;
        for( int col_shift = -max_nn_half_width_; col_shift <= max_nn_half_width_; col_shift++){
            PointT pt_nn = cloud_in->at(col+col_shift, row+row_shift);
            
            if (std::abs(col_shift) <= min_nn_half_width_ || pcl::squaredEuclideanDistance(pt_row_mid, pt_nn) < max_row_nn_dist_sqrd_){
                cloud_nn->points[cloud_nn_idx] = pt_nn;
                cloud_nn_idx++;
                num_pts_row++;
            }
        }
        if (num_pts_row < min_nn_half_width_){ return false; }
    }
    cloud_nn->points.resize(cloud_nn_idx);
    return true;   
}

int ApproxNormalEstimation::getNumNearestNeighborOutliers(PointT pt, PointCloud::Ptr cloud_nn, Eigen::Vector3f pt_normal){
    int num_outliers = 0;
    for (size_t i = 0; i < cloud_nn->points.size(); i++){
        PointT pt_nn = cloud_nn->points[i];
        float normal_dist = fabs((pt.x-pt_nn.x)*pt_normal[0] + (pt.y-pt_nn.y)*pt_normal[1] + (pt.z-pt_nn.z)*pt_normal[2]);
        if (normal_dist > max_valid_nn_normal_dist_){ num_outliers++; } 
    }
    return num_outliers;
}

void ApproxNormalEstimation::computeSurfaceNormals (const PointCloud::Ptr cloud_in, PointCloudNormal::Ptr cloud_normals){
    Eigen::Matrix3f eigen_vecs;
    Eigen::Vector3f eigen_vals;    
    PointCloud::Ptr cloud_nn (new PointCloud ());
    cloud_normals->points.reserve(cloud_in->height * cloud_in->width);
    for( size_t row = 0; row < cloud_in->height; row += row_downsample_factor_){
        for( size_t col = 0; col < cloud_in->width; col += col_downsample_factor_){
            PointT pt = cloud_in->at(col,row);
            PointNormal pt_normal = createNaNXYZNormalPoint();
            if (std::isfinite(pt.x)) {
                if (getNearestNeighborCloud(cloud_in, row, col, cloud_nn)){
                    Eigen::Vector3f approx_normal;
                    if (getApproxNormal(cloud_in, row, col, approx_normal)){
                        int num_nn_outliers = getNumNearestNeighborOutliers(pt, cloud_nn, approx_normal); 
                        if (num_nn_outliers == 0){
                            float pt_dot_normal = pt.x*approx_normal[0] + pt.y*approx_normal[1] + pt.z*approx_normal[2];
                            float pt_to_normal_ang = acos(fabs(pt_dot_normal)/sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z));
                            if (pt_to_normal_ang < max_pt_to_normal_ang_){
                                if (pt_dot_normal > 0){
                                    approx_normal *= -1;
                                }
                                pt_normal.x = pt.x;
                                pt_normal.y = pt.y;
                                pt_normal.z = pt.z;
                                pt_normal.normal_x = approx_normal[0];
                                pt_normal.normal_y = approx_normal[1];
                                pt_normal.normal_z = approx_normal[2];
                                pt_normal.curvature = eigen_vals[0];
                                cloud_normals->points.push_back(pt_normal);
                            }
                        }
                    } 
                }
            }
        }
    }
}

