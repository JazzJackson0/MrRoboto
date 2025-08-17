#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <regex>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include "kd_tree.hpp"

using std::pair;
using namespace Eigen;

struct PointCloud {
	std::vector<VectorXf> points;
	std::vector<float> weights;
	float mean_x;
	float mean_y;
	KDTree kd_tree;
	PointCloud() : kd_tree(2) {}
};

struct RotationTranslation {
	MatrixXf rotation_matrix;
	VectorXf translation_vector;
	VectorXf center_mass;
	float weight;
};

struct ControlCommand {
	float trans_vel; 
	float rot_vel;
};

typedef pair<float, VectorXf> AngleAndAxis;

namespace utils {

/**
 * @brief Convert 3D rotation matrix to rotation angle and axis of rotation pair
 * 
 * @param R Rotation matrix
 * @return AngleAndAxis - rotation angle and axis
 */
AngleAndAxis rotationMatrix3DToAngle(MatrixXf R);


/**
 * @brief Convert 2D rotation matrix to rotation angle
 * 
 * @param R Rotation matrix
 * @return float 
 */
float rotationMatrix2DToAngle(MatrixXf R);


/**
 * @brief Convert rotation angle and axis to Rotation matrix
 * 
 * @param a_a rotation angle and axis
 * @return MatrixXf - Return rotation matrix
 */
MatrixXf angleTo3DRotationMatrix(AngleAndAxis a_a);


/**
 * @brief Calculate the Greatest Common Denominator between two given denominators
 * 
 * @param a First denominator
 * @param b Second denominator
 * @return int Greatest Common Denominator
 */
int gcd (int a, int b);

/**
 * @brief Return the largest value between the two numbers
 * 
 * @param a First number
 * @param b Second number
 * @return int 
 */
int max(int a, int b);

/**
 * @brief Return the smallest value between the two numbers
 * 
 * @param a First number
 * @param b Second number
 * @return int 
 */
int min(int a, int b);


/**
 * @brief 
 * 
 * @param s 
 * @param regx 
 * @return std::vector<std::string> 
 */
std::vector<std::string> split(const std::string& s, std::string regx);


/**
 * @brief Normalize radian angle to 
 * 
 * @param angle Angle to normalize
 * @param pi_to_pi Indicate normalization range: 
 * 			False -> Range [0, 2PI); 
 * 			True -> Range [-PI, PI);
 * @return double 
 */
double normalizeAngleRadians(double angle, bool pi_to_pi);

}



