#pragma once
#include <iostream>
#include <random>
#include <chrono>
#include <time.h>
#include <vector>
#include <cmath>
#include "utils.hpp"
#include <Eigen/Dense>
#include "unsupported/Eigen/CXX11/Tensor"
using std::vector;
using std::pair;
using std::make_pair;
using namespace Eigen;



struct Particle {
	VectorXf pose;
	float weight;
};

struct Distribution {
	float mean;
	float std_dev;
	Distribution(float _mean, float _std_dev) : mean(_mean), std_dev(_std_dev ) {}
};

class ParticleFilter {

    private:
		std::vector<Particle> ParticleSet;
		float MaxBeamDist;
		float AngularBeamWidth;
		int MaxParticles;
		int PoseDimensions;
		float TimeInterval;
		float pose_sigma; // Pose Standard Deviation
		float range_sigma; // Standard Deviation for the Range
		float bearing_sigma; // Standard Deviation for the Bearing
		float range_coef; // How much range weight impacts total particle weight
		float bearing_coef; // How much bearing weight impacts total particle weight
		
		
		// Map
		Eigen::Tensor<float, 2> Map;
		int MapWidth;
		int MapHeight;
        

		/**
		 * @brief 
		 * 
		 * @param p1 
		 * @param p2 
		 * @return float 
		 */
		float get_range(VectorXf p1, VectorXf p2);


		/**
		 * @brief 
		 * 
		 * @param p1 
		 * @param p2 
		 * @return * float 
		 */
		float get_bearing(VectorXf p1, VectorXf p2);


		/**
		 * @brief 
		 * 
		 * @param lowVal 
		 * @param highVal 
		 * @return float 
		 */
		float get_random_between(float lowVal, float highVal);


		/**
		 * @brief Returns the probability of x occurring. A higher value (used as a weight) indicates a higher similarity 
		 * 			between robot & particle data. A lower value indicates a low similarity.
		 *
		 * @param x 
		 * @param distribution Particle Data
		 *
		 * @return ** float - Probability
		 */
		float probability_density_function(float x, Distribution distribution);


		/**
		 * @brief Pseudo Scan. Get all points within range of particle.
		 * 
		 * @param particle The particle around which feature points will be checked for.
		 * 
		 * @return ** Scan - Returns a vector of all the feature points picked up within range of the give particle
		 */
		PointCloud get_feature_points_in_range(Particle particle);


		/**
		 * @brief 
		 * 
		 * @param particle_idx 
		 * @param odom 
		 * @return Particle 
		 */
		Particle move_particle(int particle_idx, ControlCommand odom);


		/**
		 * @brief Generates a weight value for a single particle by comparing the similarity between the point cloud from 
		 * 		the scanner and the estimated point cloud for the particle at 'particle_idx' 
		 * 
		 * @param robot_pointcloud The point cloud obtained from the scanner.
		 * @param particle The particle to be weighted. 
		 */
		void generate_particle_weight(PointCloud robot_pointcloud, Particle &particle);

		
		/**
         * @brief Runs the particle generation and importance weighting.
		 *
		 * @param scan Robot Scan
		 * @param odom The Odometry output (v, w)
         * 
         * @return ** void
         */
        void run_particle_filter(PointCloud scan, ControlCommand odom);


        /**
         * @brief Uses the Low Variance Resampling Algorithm to resample the particles.
         * 
         * @return ** void 
         */
        void resample();



    public:

		/**
		 * @brief Default Constructor
		 * 
		 */
		ParticleFilter();

        /**
         * @brief Initializes the particle filter and creates a uniform distribution of particles.
		 *
		 * @param max_particles Max Number of Particles 
		 * @param pose_dimensions Robot/Particle pose dimensions
		 * @param time_interval Time interval ..........................
		 * @param search_dist The distance from the particle to search for map features
		 * @param search_width The Width of the range around the particle to search for map features
         */
        ParticleFilter(int max_particles, int pose_dimensions, float time_interval);

		/**
		 * @brief Add the map that the Particle Filter will localize the robot in.
		 * 
		 * @param map Map used to localize the robot in.
		 * @param max_beam_dist The distance from the particle to search for map features
		 * @param angular_beam_width The Width of the range around the particle to search for map features
		 */
		void addMap(Eigen::Tensor<float, 2> map, float max_beam_dist, float angular_beam_width);


		/**
		 * @brief Run the Particle Filter (Monte Carlo Localization) Algorithm.
		 * 
		 * @param scan 
		 * @param odom 
		 */
		void run(PointCloud scan, ControlCommand odom);
};





