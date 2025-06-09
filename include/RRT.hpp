#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <limits>
#include <utility>
#include <vector>
#include <list>
#include <stack>
#include <Eigen/Dense>
#include "unsupported/Eigen/CXX11/Tensor"
#include "Graph.hpp"
using std::make_pair;
using namespace Eigen;

struct RRT_Node {	
	int x;
	int y;
	int dist_from_start;
	int parent_idx;
    RRT_Node() {}
    RRT_Node(int _x, int _y) : x(_x), y(_y) {}
    bool operator == (RRT_Node otherNode) {
        return (this->x == otherNode.x && this->y == otherNode.y);
    }
    void setPoint(int x, int y) {
        this->x = x;
        this->y = y;
    }
};


struct RRT_Edge {

    float distance;
    int parent_index;
    int child_index;
    RRT_Edge() {}
    RRT_Edge(float _distance, int _parent_index, int _child_index) 
        : distance(_distance), parent_index(_parent_index), child_index(_child_index) {}
};

class RRT {

    private:
        int HEIGHT;
        int WIDTH;
        Eigen::Tensor<float, 2> MAP;
        Graph<RRT_Node, RRT_Edge> RapidTree;
		float SearchRadius;
		RRT_Node Start;
		RRT_Node Goal;
        std::random_device rd;
        bool repeat_node;
        int MaxConnectionDistance; // Maximum distance that a Vertex can be from another Vertex it's connected to.


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool is_valid(int x, int y);


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool is_blocked(int x, int y);


        /**
         * @brief 
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool is_goal_reached(int x, int y);


        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool is_start_and_goal_valid();


        /**
         * @brief 
         * 
         * @param node 
         * @param nearest 
         * @return true 
         * @return false 
         */
        bool is_visible(RRT_Node node, RRT_Node nearest);


        /**
		 * @brief Calculate the Euclidean Distancee between two Nodes
		 *
		 * @param node_a Node A
		 * @param node_b Node B
		 *
		 * @return ** float - The distance between 2 nodes.
		 * **/
		float get_distance(RRT_Node node_a, RRT_Node node_b); 


        /**
         * @brief Produce a random (x, y) coordinate pair within the bounds of the grid.
         * 
         * @return ** Node - A Random (x, y) position on the grid.
         */
        RRT_Node get_random_position();


        /**
         * @brief Produce a vector of all nodes within a given radius of the node.
		 *
		 * @param nodes The node at the center of the search radius.
         * @param search_radius The radius around the given node in which to search for nodes.
         * @param neighbors
         * @return true 
         * @return false 
         */
		bool get_neighbors(RRT_Node node, float search_radius, std::vector<int> &neighbors); 


        /**
         * @brief Find the Vertex in the Tree that is nearest to the given node.
         * 
         * @param randPos node
		 *
         * @return ** pair<int, float> - (Index of the Nearest Vertex, Distance to node)
         */
        std::pair<int, float> get_nearest_vertex_index(RRT_Node node);


        /**
         * @brief 
         * 
         * @param node 
         * @param nearest
         * @return bool 
         */
        bool move_node_closer(RRT_Node &node, RRT_Node nearest);

        
        /**
         * @brief Set new Vertex at the Max Connection Distance in the direction of the 
         *          Random Coordinate. 
         * 
         * @param nearest_info A pair: (Vertex Index within Graph, Distance from Random Position).
         * @param new_vertex_data Data to add to to new vertex
         * @return true - If new Vertex was set.
         * @return false - If new Vertex was not set.
         */
        bool connect_new_vertex(std::pair<int, float> nearest_info, RRT_Node new_vertex_data);
		
		
		/**
		 * @brief Reconnects a given node to whichever one produces the shortest path 
         *          back to the start.
		 *
		 * @param neighbors List of vertices within the search radius of a random node.
		 * @param newest_node_idx The node to rewire
		 *
		 * @return ** void 
		 */
		void rewire_neighbors(std::vector<int> neighbors, int newest_node_idx);


        void update_distances(int node_index);


        std::stack<VectorXi> path_trace_helper(int goal_node_idx, int current_node_idx, std::stack<VectorXi> path);

        std::vector<VectorXi> path_trace(int goal_node_idx, int current_node_idx);


    public:

        /**
         * @brief Default Constructor
         * 
         */
        RRT();

        /**
         * @brief Performs either an RRT or an RRT* search on a given Grid.
         * 
         * @param map The Grid that will be searched
         * 
         */
        RRT(Eigen::Tensor<float, 2> map);


        void loadMAP(Eigen::Tensor<float, 2> map);


        /**
         * @brief Runs the RRT algorithm
         * 
         * @param start Start Coordinates
         * @param goal Goal Coordinates
         * @param maxConnectionDistance Maximum distance that a Vertex can be from another Vertex 
         *                          it's connected to.
         * @return ** std::vector<VectorXf> - The waypoints of the path  
         */
        std::vector<VectorXi> rrtPath(VectorXi start, VectorXi goal, float maxConnectionDistance);

		/**
		 * @brief Runs an optimized version of RRT that provides a shorter path to the goal than
         *          RRT.
		 *
		 * @param start Start Coordinates
		 * @param goal Goal Coordinates
		 * @param maxConnectionDistance The max distance nodes are allowed to be from each other.
		 * @param search_radius The search radius distance.
		 *
		 * @return ** std::vector<VectorXf> - The waypoints of the path 
		 */
        std::vector<VectorXi> rrtStarPath(VectorXi start, VectorXi goal, float maxConnectionDistance, float search_radius);
};





