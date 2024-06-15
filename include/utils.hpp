#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <regex>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/src/Core/Matrix.h>

using std::pair;
using namespace Eigen;


struct PointCloud {
	std::vector<VectorXf> points;
	std::vector<float> weights;
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


/**
 * @brief Convert rotation matrix to rotation angle and axis of rotation pair
 * 
 * @param R Rotation matrix
 * @return AngleAndAxis - rotation angle and axis
 */
AngleAndAxis RotationMatrix_to_Angle(MatrixXf R);


/**
 * @brief Convert rotation angle and axis to Rotation matrix
 * 
 * @param a_a rotation angle and axis
 * @return MatrixXf - Return rotation matrix
 */
MatrixXf Angle_to_RotationMatrix(AngleAndAxis a_a);


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