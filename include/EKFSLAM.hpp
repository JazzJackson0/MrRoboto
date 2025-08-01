#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <omp.h>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include "unsupported/Eigen/CXX11/Tensor"
#include <cppad/cppad.hpp>

#include "FeatureExtraction.hpp"
#include "MapBuilder.hpp"
#include "utils.hpp"

#define DELTA 1.0 // 0.005
#define EPSILLON 2.0 // 0.5
#define GAP_VAL 0.15
#define MIN_SEED_SEG_NUM 9

using namespace CppAD;
using namespace Eigen;
using std::vector;


enum f_type {PREDICTION, OBSERVATION};

// Why did I make these. What are they even from???????????????????????????????????????????????
typedef struct state_vec StateVec;
typedef struct pose_plus PosePlus;
typedef enum f_type FunctionType;


class EKFSlam {

    private:
        FeatureExtractor feature_extractor;
		MapBuilder map_builder;
		Eigen::Tensor<float, 2> map_structure;
		Eigen::Tensor<float, 2> map_structure_mask;
		const int VIEW_RANGE = 600; // cm
		
		VectorXf StateVector; // The vector containing the current robot pose and all landmark positions.
		std::vector<VectorXf> Landmarks;
        Eigen::MatrixXf PredictionMappingFunction_F;
		Eigen::MatrixXf ObservationMappingFunction_F;
		Eigen::MatrixXf Covariance;
		Eigen::MatrixXf ProcessNoiseCovariance_R;
		Eigen::MatrixXf MeasurementCovariance_Q;
		float process_uncertainty_r;
		float measurement_uncertainty_q;
		float time_interval;

		// Dimensions
		int PoseDimensions;
		int LandmarkDimensions;
		int NumOfLandmarks; // current Number of Landmarks in the Map
		int MaxLandmarks;
		std::vector<Landmark> Correspondence;
		float SimilarityMargin;

		// 
		Eigen::MatrixXf HighDimension_G;
		Eigen::MatrixXf HighDimension_H;
		Eigen::MatrixXf KalmanGain;
		Eigen::MatrixXf Identity;
		
		VectorXf InitialPosition;
		VectorXf PreviousPose;
		std::vector<AD<float>> Xg;
		std::vector<AD<float>> Yg;
		std::vector<AD<float>> Xh;
		std::vector<AD<float>> Yh;
		ADFun<float> PredictionFunction;		
		ADFun<float> ObservationFunction;	

		bool initial_state_set;
		bool map_state_set;

		bool non_linear;
		int map_height;
		int map_width;


		/**
		 * @brief Uses a BFS to propagate accross the map, marking the spaces that the robot can see as "free"
		 * 
		 */
		void propagate_free_space();

		/**
		 * @brief Checks if a given cell is visible from the robot
		 * 
		 * @param x 
		 * @param y 
		 * @param x_robot 
		 * @param y_robot 
		 * @return true 
		 * @return false 
		 */
		bool is_visible(int x, int y, int x_robot, int y_robot);

		/**
		 * @brief Create a Map object
		 * 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> update_map();


		/**
		 * @brief Builds the INITIAL State Vector.
		 *
		 * @return ** void 
		 */
		void build_state_vector();


		/**
		 * @brief Builds the INITIAL Covariance Matrix.
		 * 
		 */
		void build_covariance();


		/**
		 * @brief Builds the Process Noise and Measurement Noise covariance matrices
		 * 
		 */
		void build_noise_covariances();


		/**
		 * @brief Builds the Observation and Update mapping function matrices
		 * 
		 */
		void build_mapping_functions();


		/**
		 * @brief Builds the Identity Matrix
		 * 
		 */
		void build_identity();

		/**
		 * @brief Add a new landmark coordinates to the map (Increasing thre size of the state vector and all related matrices),
		 * 			and add landmark to Correspondences.
		 * 
		 * @param landmark 
		 * @return int 
		 */
		int update_map_and_resize(Landmark landmark);


		/**
		 * @brief Predicts the pose based on a given mathematical motion model / Update Function g().
		 *
		 * @param ctrl Odometry reading (translation velocity & rotation velocity) 
		 *
		 * @return ** VectorXf Updated Pose
		 */
		VectorXf predict_pose_g(ControlCommand ctrl);


		/**
		 * @brief Creates a Landmark Range & Bearing Estimation based on a given mathematical model / Observation Function h().
		 * 				|||  This function currently assumes a 2D pose and 2D landmark position.
		 *
		 * @param robot_pose_est Estimated Robot Pose (the output of the Update Pose function)
		 * @param landmark_position_est Estimated Landmark Position (Obtained from......................)
		 *
		 * @return ** VectorXf Updated Landmark Data
		 */
		VectorXf get_estimated_landmark_h(VectorXf robot_pose_est, Point landmark_position_est);


		/**
		 * * @brief Sets up the Prediction Function/Motion Model g() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix G.
		 *
		 * @return ** void
		 */
		void build_prediction_function_for_G();


		/**
		 * @brief Sets up the Observation Function h() as a true set of equations 
		 * 			with unknown independent variables for input into the Jacobian Matrix H.
		 *
		 * @return ** void
		 */
		void build_observation_function_for_H();


		/**
         * @brief Creates and solves the Jacobian for a given function.
         * 
         * @param f_type The function to create and solve a Jacobian for.
		 * 					|||  UPDATE: Update Function, OBSERVATION: Observation Function
		 * 
         * @return ** MatrixXf - Jacobian Matrix 
         */
		MatrixXf calculate_jacobian(FunctionType f_type, int landmark_location);


		/**
         * @brief Update the Mean with a new Predicted State and propagate the Covariance Matrix
         *          forward in time.
		 *
		 * @param ctrl Odometry reading (translation velocity & rotation velocity) 
         * 
         * @return ** void 
         */
        void prediction(ControlCommand ctrl);



        /**
         * @brief Correct the Predicted state using the given Input data and 
         *          a Kalman Gain weight factor applied to the given estimate.
		 * 
		 * @param current_scan The current range scan.
         * 
         * @return ** void 
         */
        void correction(std::vector<Landmark> current_scan);
    
	public:

		/**
		 * @brief Default constructor
		 * 
		 */
		EKFSlam();

        /**
         * @brief Initialize an EKF Slam object
		 *
		 * 
		 * @param pose_dim Pose Dimensions
		 * @param landmark_dim Landmark Dimensions
         */
        EKFSlam(int pose_dim, int landmark_dim);

		/**
		 * @brief Set the Initial State 
		 * 
		 * @param initial_position Starting position of the robot
		 * @param _process_uncertainty_r A constant that corresponds to the non-zero diagonal components of R (dim = dim(pose))
		 * @param _measurement_uncertainty_q A constant that corresponds to the non-zero components of Q (The uncertainty of each landmark)
		 */
		void setInitialState(Eigen::VectorXf initial_position, float _process_uncertainty_r, float _measurement_uncertainty_q);

       
		/**
		 * @brief Run the EKF SLAM Algorithm
		 * 
		 * @param current_scan 
		 * @param ctrl 
		 * @return Eigen::Tensor<float, 2> 
		 */
        Eigen::Tensor<float, 2> run(PointCloud current_scan, ControlCommand ctrl);


		/**
		 * @brief Set the landmark data if correspondences are known.
		 * 
		 * @param landmarks A vector of all known landmark (x, y) positions.
		 */
		void setKnownLandmarks(std::vector<VectorXf> landmarks);

		/**
		 * @brief 
		 * 
		 * @param height 
		 * @param width 
		 */
		void setMapDimensions(int height, int width);


		/**
		 * @brief 
		 * 
		 * @return VectorXf 
		 */
		VectorXf broadcastCurrentPose();


		
};


