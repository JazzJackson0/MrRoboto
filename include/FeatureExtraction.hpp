#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include "utils.hpp"

using std::pair;
using std::vector;
using namespace Eigen;

struct Point {

    float x;
    float y;
    float angle;
    int err;
};

struct GeneralFormLine {

    double a;
    double b;
    double c;
    int err;
};

struct SlopeInterceptLine{

    double m;
    double b;
    int err;
};

struct LineSegment {

    GeneralFormLine general_fit_line;
    SlopeInterceptLine slope_fit_line;
    std::vector<Point> points;
    std::vector<Point> predicted_points;
    int start_idx;
    int end_idx;
    std::vector<Point> endpoints;
    int err;
};

struct Landmark {

    int id;
    Point position;
    float range; // Range from robot
    float bearing; // Bearing relative to robot
    LineSegment line_seg;
    int err;
};


class FeatureExtractor {

    private:
        int MinSeedSegNum; // 
        int MinLineSegNum; // 
        float MinLineSegLen; // 
        int SeedSegWindowSize;
        float Delta; // Distance (in meters) threshold from point position to predicted point position
        float Epsillon; // Distance (in meters) threshold from every potential segment point to the fitting line
        float GapValue; // Acceptable distance between points
        Point RobotPos; // Current Robot Position
        int LandmarkIDs;
        std::vector<Point> LaserPoints;
        std::vector<Landmark> NewLandmarks;
        std::vector<Landmark> AllLandmarks;
        int breakpoint_idx;
        

        /**
         * @brief Calculate euclidean distance between two points
         * 
         * @param point_a 
         * @param point_b 
         * @return float 
         */
        float get_euclidean_distance(Point point_a, Point point_b);

        /**
         * @brief Calculate distance between point and line
         * 
         * @param point 
         * @param general_line 
         * @return float 
         */
        float get_point_to_line_distance(Point point, GeneralFormLine general_line);

        /**
         * @brief Get two points in a given line
         * 
         * @param x1 The x value used to calculate the first point
         * @param x2 The x value used to calculate the second point
         * @param slope_line The line to get the points from
         * @return vector<Point> The two points
         */
        std::vector<Point> get_2_points_from_line(int x1, int x2, SlopeInterceptLine slope_line);

        /**
         * @brief Converts Line from Slope-Intercept Form to General Form
         * 
         * @param slope_line line in slope-intercept form
         * @return GeneralFormLine - line in general form
         */
        GeneralFormLine slope_int_to_general(SlopeInterceptLine slope_line);


        /**
         * @brief Converts Line from General Form to Slope-Intercept Form
         * 
         * @param general_line line in general form
         * @return SlopeInterceptLine - line in slope-intercept form
         */
        SlopeInterceptLine general_to_slope_int(GeneralFormLine general_line);

        /**
         * @brief Calculate the intersection between 2 lines
         * 
         * @param general_line_1 line 1 (in general form)
         * @param general_line_2 line 2 (in general form)
         * @return Point - Intersection point
         */
        Point get_intersection(GeneralFormLine general_line_1, GeneralFormLine general_line_2);

        /**
         * @brief Get Position Coordinate from Angle & Distance Information
         * 
         * @param dist distance measurement
         * @param angle angle measurement
         * @return Point 
         */
        Point angle_distance_to_position(float dist, float angle);

        /**
         * @brief Transform scan from array of range & bearing values to array of position coordinates
         * 
         * @param scan point cloud
         * @return vector<Point> 
         */
        std::vector<Point> transform_scan(PointCloud scan);

        /**
         * @brief Create a Linear Model given two points
         * 
         * @param point_1 
         * @param point_2 
         * @return SlopeInterceptLine 
         */
        SlopeInterceptLine create_linear_model(Point point_1, Point point_2);


        /**
         * @brief Fit line to set of points using Orthogonal Distance Regression (i.e. Orthogonal line fitting)
         * 
         * @param laser_points Set of points
         * @return GeneralFormLine 
         */
        GeneralFormLine odr_fit(std::vector<Point> laser_points);

        /**
         * @brief Calculate the predicted position of a given point by calculating the intersection 
         *          between its (range & bearing) and the fitted line.
         * 
         * @param fitted_line The line parameters for the fitted line (e.g. ODR) that will intersect with 
         *              the calculated beam line in order to get the predicted point.
         * @param point_in_scan The geven point from the scan
         * @return Point - The predicted point
         */
        Point get_point_prediction(GeneralFormLine fitted_line, Point point_in_scan);


        /**
         * @brief Calculate the endpoints for a seed segment
         * 
         * @param line All data about the segment
         * @param point_a The outermost point on end a
         * @param point_b The outermost point on end b
         * @return vector<Point> The two endpoints
         */
        std::vector<Point> get_endpoints(LineSegment line, Point point_a, Point point_b);
        
        
        /**
         * @brief Calculate the Orthogonal Projection of a given point to a given line
         * 
         * @param line 
         * @param data_point 
         * @return Point 
         */
        Point orthog_project_point_to_line(LineSegment line, Point data_point);


        /**
         * @brief Take a point on a line and clamp it to between the corresponding seed segment's endpoint values.
         * 
         * @param line 
         * @param point 
         * @return Point 
         */
        Point clamp_point_on_line(LineSegment line, Point point);


        /**
         * @brief Compare the landmarks obtained from the curent scan to all previously saved landmarks.
         *       Determine if the new landmarks are ones previously seen (upon which you update the old
         *      version of it with the new one) or new (upon which you add that landmark to the saved ones).
         * 
         */
        void check_overlap();


        /**
         * @brief Detect seed segment from current set of points
         * 
         * @return SeedSegment 
         */
        LineSegment detect_seed_segment();


        /**
         * @brief Grow the given seed segment
         * 
         * @param seed_seg 
         * @return LineSegment 
         */
        LineSegment grow_seed_segment(LineSegment seed_seg);


        /**
         * @brief Validates the given feature by outputting it as a landmark, provided
         *          it passes validation requirements
         * 
         * @param feature line segment to validate as a landmark
         * @return Landmark Unvalidated landmarks will have an error value of 1
         */
        Landmark validation_gate(LineSegment feature);


        /**
         * @brief Reset all global variables needed to find new landmarks 
         * 
         */
        void reset();


    public:

        // Default construtor
        FeatureExtractor();

        /**
         * @brief Construct a new Feature Extractor object
         * 
         * @param delta Distance threshold from point position to predicted point position
         * @param epsillon Distance threshold from every potential segment point to the fitting line
         * @param gap_value Acceptable distance between points
         * @param min_seed_seg_num 
         */
        FeatureExtractor(float delta, float epsillon, float gap_value, int min_seed_seg_num);

        /**
         * @brief Runs the Feature Extraction Algorithm, pulling all Landmarks out of the current scan
         * 
         * @param current_scan 
         * @param current_pose
         */
        std::vector<Landmark> landmarksFromScan(PointCloud current_scan, VectorXf current_pose);


        /**
         * @brief Set new delta value
         * 
         * @param delta Distance threshold from point position to predicted point position
         */
        void setDelta(float delta);


        /**
         * @brief Set new epsillon value
         * 
         * @param epsillon  Distance threshold from every potential segment point to the fitting line
         */
        void setEpsillon(float epsillon);


        /**
         * @brief 
         * 
         * @param gap_val 
         */
        void setGapValue(float gap_val);


        /**
         * @brief 
         * 
         * @param min_seed_seg_num 
         */
        void setMinSeedSegNum(int min_seed_seg_num);


        /**
         * @brief Set minimum line segment length threshold
         * 
         * @param min_line_seg_len 
         */
        void setMinLineSegLen(float min_line_seg_len);

};


