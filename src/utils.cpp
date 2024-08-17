#include "../include/utils.hpp"




AngleAndAxis RotationMatrix3D_to_Angle(MatrixXf R) {
    
    
    if (R.cols() != 3 && R.rows() != 3) {
        std::cerr << "Rotation Matrix dimensions should be 3x3. Cannot convert to angle." << std::endl;
        VectorXf fail_axis(3);
        fail_axis << -1, -1, -1;
        float fail_angle = -1;
        return std::make_pair(fail_angle, fail_axis);
    }

    float angle = (float) acos((R.trace() - 1) / 2);
    VectorXf rot_axis(3);
    VectorXf R_diff(3);
    R_diff << (R(2, 1) - R(1, 2)), (R(0, 2) - R(2, 0)), (R(1, 0) - R(0, 1));

    float sinAngle = sqrt(1 - pow((R.trace() - 1 / 2), 2));
    

    // Singularity
    if (angle == 0.0) {

        rot_axis << 0, 0, 1;
    }

    // Singularity (angle is 180 or very close to it)
    else if (sinAngle == 1e-6) {
        
        // Find diagonal term with highest value
        float largest = std::numeric_limits<float>::min();;
        int row = -1;

        for (int i = 0; i < R.cols(); i++) {

            if (R(i,i) > largest) {
                row = i;
                largest = R(i,i);
            }
        }

        // Calculate the axis vector
        float v_a = 0.5 * sqrt(max(0, (1 + R(0, 0) - R(1, 1) - R(2, 2))));

        if (largest == R(0, 0)) {
            rot_axis << v_a, ((R(0, 1) + R(1, 0)) / (2 * v_a)), ((R(0, 2) + R(2, 0)) / (2 * v_a));
        }

        else if (largest == R(1, 1)) {
            rot_axis << ((R(0, 1) + R(1, 0)) / (-2 * v_a)), v_a, ((R(0, 2) + R(2, 0)) / (-2 * v_a));
        }

        else {
            rot_axis << ((R(0, 2) + R(2, 0)) / (-2 * v_a)), ((R(1, 2) + R(2, 1)) / (-2 * v_a)), v_a;
        }
    }

    else {
        rot_axis = (1 / (2 * sin(angle))) * R_diff;
    }

    // Normalize
    rot_axis = rot_axis / (sqrt(rot_axis.dot(rot_axis)));

    return std::make_pair(angle, rot_axis);
}


float RotationMatrix2D_to_Angle(MatrixXf R) {
    
    
    if (R.cols() != 2 && R.rows() != 2) {
        std::cerr << "Rotation Matrix dimensions should be 2x2. Cannot convert to angle." << std::endl;
        return -1;
    }

    return atan2(R(1, 0), R(0, 0));
}


MatrixXf Angle_to_3DRotationMatrix(AngleAndAxis a_a) {

    // Rodriguez Formula method
    // MatrixXf identity;
    // identity = MatrixXf::Identity(3, 3);

    // if (a_a.first == 0.0) {

    //     return identity;
    // }
    
    // // Convert axis angle to matrix form
    // MatrixXf A(4, 4);
    // A << 0, -a_a.second(2), a_a.second(2), a_a.second(2), 0, -a_a.second(0), -a_a.second(1), a_a.second(0), 0;

    // return identity - (sin(a_a.first) * A) + ((1 - cos(a_a.first)) * (A * A));

    MatrixXf R(2, 2);
    R << cos(a_a.first), -sin(a_a.first), sin(a_a.first), cos(a_a.first);
    return R;
}

int gcd (int a, int b) {

    if (b == 0)
        return a;

    else 
        return gcd(b, a % b);
}

int max(int a, int b) {

    if (a > b)
        return a;
    
    else
        return b;
}


int min(int a, int b) {

    if (a < b)
        return a;
    
    else
        return b;
}

std::vector<std::string> split(const std::string& s, std::string regx) {
    
    std::vector<std::string> elems;
    std::regex re(regx); 
    std::sregex_token_iterator iter(s.begin(), s.end(), re, -1);
    std::sregex_token_iterator end;
    
    while (iter != end) {
        if (iter->length()) { elems.push_back(*iter); }
        ++iter;
    }

  return elems;
}



double normalizeAngleRadians(double angle, bool pi_to_pi) {
    
    if (std::isnan(angle) || std::isinf(angle))
        return std::numeric_limits<double>::quiet_NaN();

    // Normalize to the range [0, 2*PI)
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
    
    if (pi_to_pi) {

        // Normalize to the range [-PI, PI)
        if (angle >= M_PI)
            angle -= 2 * M_PI;
    }
    
    return angle;
}