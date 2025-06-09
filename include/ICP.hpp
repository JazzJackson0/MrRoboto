#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <limits>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>

#include <cppad/cppad.hpp>
#include "utils.hpp"

using std::vector;
using std::pair;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace::CppAD;
using namespace::Eigen;

class ICP {
	
	private:

		int PoseDimension;
		int ErrorDimension;
		int ErrorParameterNum = 3;
		std::vector<AD<float>> X; // X = (t_x, t_y, angle)
		std::vector<AD<float>> Y;
		ADFun<float> ErrorFunction;
		float min_convergence_thresh;	

		/**
		 * @brief Calculate the root mean squared error between point sets.
		 * 		This value is meant to be compared to the ICP Least Squares convergence threshold.
		 * 
		 * @param RefPointSet 
		 * @param NewPointSet 
		 * @return float RMS Error
		 */
		float get_root_mean_squared_error(PointCloud RefPointSet, PointCloud NewPointSet);

		/**
		 * @brief Calculate the Euclidean Distance between two points.
		 * 
		 * @param p 
		 * @param q 
		 * @return float 
		 */
		float get_euclidean_distance(VectorXf p, VectorXf q);

		/**
		 * @brief Calculate the Center of Mass for a given Point Cloud
		 * 
		 * @param p_cloud The cloud you calculate the center of mass of
		 * @return VectorXf Center of Mass
		 */
		VectorXf get_center_of_mass(PointCloud p_cloud);


		/**
		 * @brief Updates a given Point Cloud with the incremental result from the solution
		 * 			to a Linear System.
		 * 
		 * @param PointCloud The Point Cloud to update
		 * @param x_increment The increment to add to the Point Cloud.
		 * @return ** PointCloud  Updated Point Cloud
		 */
		PointCloud update_point_cloud(PointCloud PointCloud, VectorXf x_increment);
		
		/**
		 * @brief Calculates the correspondences between two point clouds and returns subsets
		 * 			of each cloud that map to each other (n-to-n).
		 *
		 * @param RefPointCloud - Reference Point Cloud
		 * @param NewPointCloud - New Point Cloud
		 *
		 * @return ** pair<PointCloud, PointCloud> - (Reference Point Set, New Point Set)
		 */
		pair<PointCloud, PointCloud>  calculate_correspondences(PointCloud RefPointCloud, 
			PointCloud NewPointCloud);

		/**
		 * @brief Get the Error between the new and reference point
		 * 
		 * @param x_param Transformation parameters (x, y, theta) applied to 'reference point set' before its comparison to 'new point set'
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 * @return ** VectorXf 
		 */
		VectorXf get_error_vector(VectorXf x_param, VectorXf ReferencePoint, VectorXf NewPoint);

		/**
		 * @brief Builds the error function for the Non-Linear Least Squares ICP method.
		 *
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 *
		 * @return ** void
		 */
		void build_error_function(VectorXf ReferencePoint, VectorXf NewPoint); 
		 

		/**
         * @brief Creates and solves the Jacobian for a given function.
		 *
		 * @param ReferencePoint Point n from Reference Point Set
		 * @param NewPoint Point n from New Point Set
		 * @param x_update
		 * 
         * @return ** MatrixXf - Jacobian 
         */
		MatrixXf calculate_jacobian(VectorXf ReferencePoint, VectorXf NewPoint, VectorXf x_update); 


	public:

		/**
		 * @brief Default constructor
		 * **/	
		ICP();

		/**
		 * @brief Initialize an Iterative Closest Point Algorithm Object.
		 *
		 * @param pose_dim Pose Dimension
		 * @param error_dim Error Vector Dimension
		 * **/	
		ICP(int pose_dim, int error_dim);



		/**
		 * @brief Run Point Cloud Registration with Known Data Association.
		 *
		 * @param RefPointSet Reference Point Set
		 * @param NewPointSet New Point Set
		 *
		 * @return ** RotationTranslation
		 */
		RotationTranslation runSVDAlign(PointCloud RefPointSet, PointCloud NewPointSet);



		/**
		 * @brief Run Point Cloud Registration with Unkown Data Association.
		 *
		 * @param RefPointCloud Reference Point Cloud
		 * @param NewPointCloud New Point Cloud
		 *
		 * @return ** RotationTranslation
		 */
		RotationTranslation runICPSVD(PointCloud RefPointCloud, PointCloud NewPointCloud);

		

		/**
		 * @brief Run Point Cloud Registration using a Non-Linear Least Squares apprroach.
		 *
		 * @param RefPointCloud Reference Point Cloud
		 * @param NewPointCloud New Point Cloud
		 * 
		 * @return ** VectorXf updated rotation & translation parameters that minimize the error between clouds.
		 */
		VectorXf runICPLeastSquares(PointCloud RefPointCloud, PointCloud NewPointCloud);
};




