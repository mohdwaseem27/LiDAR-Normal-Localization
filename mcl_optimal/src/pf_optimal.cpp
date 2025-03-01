#include "pf_optimal.h"
#include <random>
#include <Eigen/Eigenvalues>
#include <limits>
#include <algorithm>
#include <stdlib.h>


// time computation
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

using Gaussian = std::normal_distribution<float>;

Eigen::Vector3f sampleZeroMean3DGaussian(Eigen::Matrix3f cov, std::default_random_engine &generator) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;
    eigen_solver.computeDirect(cov, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigen_vals = eigen_solver.eigenvalues();
    Eigen::Matrix3f eigen_vecs = eigen_solver.eigenvectors();

    Gaussian distribution_pc1(0, sqrt(std::max(0.0f, eigen_vals(0))));
    Gaussian distribution_pc2(0, sqrt(std::max(0.0f, eigen_vals(1))));
    Gaussian distribution_pc3(0, sqrt(std::max(0.0f, eigen_vals(2))));
    
    Eigen::Vector3f principle_components_sample(distribution_pc1(generator), distribution_pc2(generator), distribution_pc3(generator));
    Eigen::Vector3f sample = eigen_vecs * principle_components_sample;
    return sample; 
}

OptimalParticleFilter::OptimalParticleFilter(PointCloudNormal::Ptr map_cloud, PFParams params) {
    num_particles_ = params.num_particles;
    init_trans_var_x_ = params.init_trans_var_x;
    init_trans_var_y_ = params.init_trans_var_y;
    init_rot_var_ = params.init_rot_var;
    motion_model_ = new MotionModel(params.odom_trans_var_per_m, params.odom_trans_var_per_rad
                                    , params.odom_rot_var_per_rad, params.odom_rot_var_per_m); 
    observation_model_ = new ObservationModel(map_cloud, params.num_observations, params.lf_obs_std_dev, params.hf_obs_std_dev
                                              , params.max_obs_nn_dist, params.max_obs_nn_ang_diff);
    initializeParticles(Eigen::Vector3f(params.init_pose_x, params.init_pose_y, 0)
                        , Eigen::Quaternionf(Eigen::AngleAxisf(params.init_pose_yaw, Eigen::Vector3f::UnitZ())));
}

void OptimalParticleFilter::initializeParticles(Eigen::Vector3f init_pos, Eigen::Quaternionf init_quat) {
    Eigen::Matrix3f init_pose_cov = Eigen::Vector3f(init_trans_var_x_, init_trans_var_y_, init_rot_var_).asDiagonal();
    particles_.clear();
    for (size_t i = 0; i < num_particles_; i++) {
        Eigen::Vector3f init_pose_noise = sampleZeroMean3DGaussian(init_pose_cov, generator_);
        Eigen::Vector3f init_pos_noise(init_pose_noise(0), init_pose_noise(1), 0);
        Eigen::Quaternionf init_quat_noise(Eigen::AngleAxisf(init_pose_noise(2), Eigen::Vector3f::UnitZ()));
         
        Eigen::Vector3f particle_pos = init_pos + init_quat * init_pos_noise;
        Eigen::Quaternionf particle_quat = init_quat * init_quat_noise;
        particles_.push_back(Particle(particle_pos, particle_quat, 1.0/num_particles_)); 
    }
    motion_model_->initializeAccumulatedMotion(Eigen::Vector3f(std::min(1.f, init_trans_var_x_), std::min(1.f, init_trans_var_y_), std::min(1.f, init_rot_var_))); 
}

void OptimalParticleFilter::setNewOdom(uint64_t new_odom_timestamp, Eigen::Vector3f new_odom_pos, Eigen::Quaternionf new_odom_quat) {
    motion_model_->setNewOdom(new_odom_timestamp, new_odom_pos, new_odom_quat);
}

void OptimalParticleFilter::filter(PointCloudNormal::Ptr cloud) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> particle_computation_times(num_particles_);

    Eigen::Vector3f motion_trans = motion_model_->getTranslation();
    Eigen::Quaternionf motion_rot = motion_model_->getRotation();
    Eigen::Matrix3d motion_noise_cov = motion_model_->getAccumulatedMotionNoiseCovariance().cast<double>();
    motion_model_->resetAccumulatedMotion();

    observation_model_->setInputCloud(cloud);
    std::vector<float> particle_log_likelihoods(num_particles_);
    for (size_t i=0; i < num_particles_; i++){
        auto particle_start_time = std::chrono::high_resolution_clock::now();
        particles_[i].updateState(motion_trans, motion_rot);
    
        Eigen::Vector3d om_dist_mean;
        Eigen::Matrix3d lf_om_dist_precision, hf_om_dist_precision;
        float om_log_likelihood_factor;
        Eigen::Vector3f pose_sample;
        if (observation_model_->getLikelihoodDistribution(particles_[i], om_dist_mean, lf_om_dist_precision, hf_om_dist_precision, om_log_likelihood_factor)) {
            Eigen::Vector3d hf_proposal_distribution_mean = (Eigen::Matrix3d::Identity() + motion_noise_cov * hf_om_dist_precision).inverse() 
                                                         * motion_noise_cov * hf_om_dist_precision * om_dist_mean;
            Eigen::Matrix3d hf_proposal_distribution_cov = motion_noise_cov * (Eigen::Matrix3d::Identity() + hf_om_dist_precision * motion_noise_cov).inverse();
            pose_sample = hf_proposal_distribution_mean.cast<float>() + sampleZeroMean3DGaussian(hf_proposal_distribution_cov.cast<float>(), generator_);

        } else {
            pose_sample = sampleZeroMean3DGaussian(motion_noise_cov.cast<float>(), generator_);
        }
        Eigen::Vector3f pose_sample_trans = Eigen::Vector3f(pose_sample(0), pose_sample(1), 0);
        Eigen::Quaternionf pose_sample_rot = Eigen::Quaternionf(1, 0, 0, 0) * Eigen::AngleAxisf(pose_sample(2), Eigen::Vector3f::UnitZ());
        particles_[i].updateState(pose_sample_trans, pose_sample_rot);

        Eigen::Vector3d lf_proposal_distribution_mean = (Eigen::Matrix3d::Identity() + motion_noise_cov * lf_om_dist_precision).inverse() 
                                                        * motion_noise_cov * lf_om_dist_precision * om_dist_mean;
        Eigen::Matrix3d lf_proposal_distribution_cov = motion_noise_cov * (Eigen::Matrix3d::Identity() + lf_om_dist_precision * motion_noise_cov).inverse();
        float om_log_likelihood_of_particle = log(sqrt(lf_om_dist_precision.determinant())/sqrt(8*pow(M_PI,3))) 
                                              - 0.5 * (lf_proposal_distribution_mean-om_dist_mean).transpose() 
                                              * lf_om_dist_precision * (lf_proposal_distribution_mean-om_dist_mean);
        float mm_log_likelihood_of_particle = log(1/sqrt(8*pow(M_PI,3)*motion_noise_cov.determinant())) 
                                              - 0.5 * lf_proposal_distribution_mean.transpose() * motion_noise_cov.inverse() * lf_proposal_distribution_mean;
        float particle_log_likelihood = om_log_likelihood_factor + om_log_likelihood_of_particle + mm_log_likelihood_of_particle 
                                        - log(1/sqrt(8*pow(M_PI,3)*lf_proposal_distribution_cov.determinant()));  
        particle_log_likelihoods[i] = particle_log_likelihood;
        // Record end time for each particle
        auto particle_end_time = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time for the particle
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(particle_end_time - particle_start_time).count();

        // Store the elapsed time for the particle
        particle_computation_times[i].push_back(elapsed_time);
    }

    //////////////////////////// SET WEIGHTS ////////////////////////////////////////

    float max_particle_log_likelihood = *std::max_element(particle_log_likelihoods.begin(), particle_log_likelihoods.end());
    float weight_sum = 0.0;
    for (size_t i=0; i < particle_log_likelihoods.size(); i++) {
        //particles_[i].setWeight(exp((particle_log_likelihoods[i] - max_particle_log_likelihood)/100000));
        particles_[i].setWeight(exp(particle_log_likelihoods[i] - max_particle_log_likelihood));
        weight_sum += particles_[i].getWeight();
    }
    std::cout << "weight SUM : " << weight_sum << std::endl;

    for (size_t i = 0; i < particles_.size(); i++) {
        particles_[i].setWeight(particles_[i].getWeight()/weight_sum); 
    }

    //////////////////////////// RESAMPLE ////////////////////////////////////////
    std::vector<float> cummulative_weights{particles_[0].getWeight()};
    for (size_t i=1; i < particles_.size()-1; i++) {
        cummulative_weights.push_back(cummulative_weights.back() + particles_[i].getWeight());
    }
    std::cout << 1.0 << std::endl;
    cummulative_weights.push_back(1.0);

    std::vector<Particle> new_particles;
    float random_prob = (float)rand()/((float)RAND_MAX+1)/num_particles_;
    size_t last_j = 0;
    for (size_t i = 0; i < num_particles_; i++) {
        for(size_t j = last_j; j < cummulative_weights.size(); j++) {
            if (cummulative_weights[j]>random_prob) {
                new_particles.push_back(particles_[j]);
                last_j = j;
                break;
            }
        }
        random_prob += 1.0/num_particles_;
    }
    auto end_time = std::chrono::high_resolution_clock::now();


    // Calculate total elapsed time for the filter function
    auto total_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Total computation time for the filter function: " << total_elapsed_time << " milliseconds" << std::endl;

    /// Write computation times to a CSV file
    std::string filename = "/home/waseem/Documents/HBRS/RnD/catkin_ws/particle_computation_times.csv";
    std::ofstream outfile(filename, std::ios::app); // Open in append mode
    if (outfile.is_open()) {
        // Write computation times for each particle as a new row
        for (size_t col = 0; col < num_particles_; ++col) {
            outfile << particle_computation_times[col][0]; // Assuming only one computation time per particle
            if (col != num_particles_ - 1) {
                outfile << ",";
            }
        }
        outfile << "\n";

        outfile.close();
        std::cout << "Computation times appended to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    particles_ = new_particles; 
}

std::vector<Particle> OptimalParticleFilter::getParticleSet() {
    return particles_;
}

Eigen::Vector3f OptimalParticleFilter::getAveragePos() {
    Eigen::Vector3f pos_sum(0, 0, 0);
    for (size_t i = 0; i < particles_.size(); i++) {
        pos_sum += particles_[i].getPos();
    }
    return pos_sum / particles_.size();
}

Eigen::Quaternionf OptimalParticleFilter::getAverageQuat() {
    Eigen::Quaternionf quat_sum(0, 0, 0, 0);
    for (size_t i = 0; i < particles_.size(); i++) {
        Eigen::Quaternionf particle_quat = particles_[i].getQuat();
        float sign = (quat_sum.norm() == 0 || quat_sum.dot(particle_quat) > 0) ? 1 : -1;
        quat_sum.x() += sign * particle_quat.x();
        quat_sum.y() += sign * particle_quat.y();
        quat_sum.z() += sign * particle_quat.z();
        quat_sum.w() += sign * particle_quat.w();
    }
    return quat_sum.normalized();
}
