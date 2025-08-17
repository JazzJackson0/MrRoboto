#include "../include/RRT.hpp"

// Private-------------------------------------------------------------------------------------------------------------------------------------

bool RRT::is_valid(int x, int y) {
    return (x >= 0) && (x < WIDTH) && 
        (y >= 0) && (y < HEIGHT);
}

bool RRT::is_blocked(int x, int y) {

    int row = y;
    int col = x;
    return MAP(row, col) != 0.f;
}


bool RRT::is_goal_reached(int x, int y) {

    return (x == Goal.x && y == Goal.y);
}


bool RRT::is_start_and_goal_valid() {
    
    if (!is_valid(Start.x, Start.y)) {
        std::cout << "ERROR: Invalid Starting Coordinates." << std::endl;
        return false;
    }

    if (!is_valid(Goal.x, Goal.y)) {
        std::cout << "ERROR: Invalid Goal Coordinates." << std::endl;
        return false;
    }

    if (is_blocked(Start.x, Start.y)) {
        std::cout << "ERROR: Start Coodrdinates Blocked." << std::endl;
        return false;
    }

    if (is_goal_reached(Start.x, Start.y)) {
        std::cout << "ERROR: Ummm... You're already there..." << std::endl;
        return false;
    }

    return true;
}


bool RRT::is_visible(RRT_Node node, RRT_Node nearest) {
    
    // Bresenham's line algorithm for line of sight checking
    int dx = std::abs(node.x - nearest.x);
    int dy = std::abs(node.y - nearest.y);
    int sx = (nearest.x < node.x) ? 1 : -1;
    int sy = (nearest.y < node.y) ? 1 : -1;
    int err = dx - dy;

    int x_curr = nearest.x;
    int y_curr = nearest.y;

    while (true) {
		// Clear line of sight
        if (x_curr == node.x && y_curr == node.y) return true;

		// Line of sight blocked by obstacle
        if (is_blocked(x_curr, y_curr) == 1.0) return false;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x_curr += sx;
        }
        if (e2 < dx) {
            err += dx;
            y_curr += sy;
        }
    }
}


float RRT::get_distance(RRT_Node node_a, RRT_Node node_b) {

	return hypot(( node_a.x - node_b.x ), ( node_a.y - node_b.y ));
}


RRT_Node RRT::get_random_position() {
    
	std::uniform_int_distribution<int> height_dist(0, HEIGHT - 1);
    std::uniform_int_distribution<int> width_dist(0, WIDTH - 1);

	int x = width_dist(rd);
    int y = height_dist(rd);
	return RRT_Node(x, y);
}


bool RRT::get_neighbors(RRT_Node node, float search_radius, std::vector<int> &neighbors) {

    int n_Vertices = RapidTree.getNumOfVertices();

    // Calculate distance between node and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {
        if (get_distance(node, RapidTree.getVertex(i)) <= SearchRadius) neighbors.push_back(i);
	}

	return neighbors.size() > 0;
}


std::pair<int, float> RRT::get_nearest_vertex_index(RRT_Node node) {
    
    float min_dist = std::numeric_limits<float>::max();
    int nearestVertexIndex = -1;
    int n_Vertices = RapidTree.getNumOfVertices();

    // Calculate distance between node and every vertex in set.
    for (int i = 0; i < n_Vertices; i++) {

		RRT_Node node_i = RapidTree.getVertex(i);

		// This random position already exists
		if (node.x == node_i.x && node.y == node_i.y) {
			repeat_node = true;
			return make_pair(-1, 0);  
		}

		float dist = get_distance(node, node_i);

        if (dist < min_dist) { 
            min_dist = dist;
            nearestVertexIndex = i;
        }
    }
    return make_pair(nearestVertexIndex, min_dist);  
}


bool RRT::move_node_closer(RRT_Node &node, RRT_Node nearest) {

	Eigen::VectorXi v(2);
	v << (node.x - nearest.x), (node.y - nearest.y);
	float v_magnitude = std::hypot(v[0], v[1]);
	VectorXf unit_vec(2);
	unit_vec << round(v[0] / v_magnitude), round(v[1] / v_magnitude);

	node.x = nearest.x + (int)(MaxConnectionDistance * unit_vec[0]);
	node.y = nearest.y + (int)(MaxConnectionDistance * unit_vec[1]);

	return is_blocked(node.x, node.y);
}


bool RRT::connect_new_vertex(std::pair<int, float> nearest_info, RRT_Node new_node) {

	float new_to_nearest = nearest_info.second;
	int nearest_id = nearest_info.first;
	RRT_Node nearest = RapidTree.getVertex(nearest_id);
	RRT_Edge null_edge;
	
    // [OPTION 1] Location is blocked--------------------------------------------------------
    if (is_blocked(new_node.x, new_node.y)) return false;

    // [OPTION 2] Random coordinate is <= Max Connect Distance: -----------------------------
    if (new_to_nearest <= MaxConnectionDistance) {
		if(!is_visible(new_node, nearest)) return false;

        // [Add new node to Graph]
		new_node.dist_from_start = new_to_nearest + nearest.dist_from_start;
		new_node.parent_idx = nearest_id;
		RapidTree.addVertex(new_node, false, null_edge);

		// [Add new edge to Graph]
		RRT_Edge edge = RRT_Edge(new_to_nearest, new_node.parent_idx, RapidTree.getNumOfVertices() - 1);
		RapidTree.addEdge(edge.child_index, edge.parent_index, edge);
        return true;
    }

    // [OPTION 3] Random coordinate is farther than Max Connect Distance:--------------------
	if(!move_node_closer(new_node, nearest)) return false;
	if(!is_visible(new_node, nearest)) return false;

	new_to_nearest = get_distance(new_node, nearest);

	// [Add new node to Graph]
	new_node.dist_from_start = new_to_nearest + nearest.dist_from_start;
	new_node.parent_idx = nearest_id;
	RapidTree.addVertex(new_node, false, null_edge);

	// [Add new edge to Graph]
	RRT_Edge edge = RRT_Edge(new_to_nearest, new_node.parent_idx, RapidTree.getNumOfVertices() - 1);	
	RapidTree.addEdge(edge.child_index, edge.parent_index, edge);
	return true;
}


void RRT::rewire_neighbors(std::vector<int> neighbors, int newest_node_idx) {

	RRT_Node new_node = RapidTree.getVertex(newest_node_idx);
	for (int i = 0; i < neighbors.size(); i++) {

		RRT_Node neighbor = RapidTree.getVertex(neighbors[i]);
		float new_to_nearest = get_distance(new_node, neighbor);
		float new_dist_from_start = new_node.dist_from_start + new_to_nearest;

		// If neighbor gets a shorter path to Start by connecting to new node.
		if (new_dist_from_start < neighbor.dist_from_start && is_visible(new_node, neighbor)) {
			
			// [Cancel Old Connection]
			RapidTree.removeEdge(neighbor.parent_idx, neighbors[i]);
			neighbor.parent_idx = newest_node_idx;
			RapidTree.updateVertexData(neighbors[i], neighbor);

			// [Create New Connection]
			RRT_Edge edge = RRT_Edge(new_to_nearest, newest_node_idx, neighbors[i]);
			RapidTree.addEdge(edge.child_index, edge.parent_index, edge);
			
			// Recalculate the 'dist_from_start' for all children of edge.child_index.
			update_distances(edge.child_index);
		}	
	}
}


void RRT::update_distances(int node_index) {

	//std::cout << "Node Index: " << node_index << std::endl;
	RRT_Node parent_node = RapidTree.getVertex(node_index);
	std::vector<RRT_Edge> edges = RapidTree.getIncidentEdges(node_index);
	
	for (int i = 0; i < edges.size(); i++) {
	
		// Update Child Node's Distance from Start
		if (edges[i].parent_index == node_index) {

			RRT_Node child_node = RapidTree.getVertex(edges[i].child_index);
			child_node.dist_from_start = parent_node.dist_from_start + edges[i].distance;
			RapidTree.updateVertexData(edges[i].child_index, child_node);
			update_distances(edges[i].child_index);
		}
	}

	return;
}



std::stack<VectorXi> RRT::path_trace_helper(int goal_node_idx, int current_node_idx, std::stack<VectorXi> path) {

	RRT_Node current_node = RapidTree.getVertex(current_node_idx);
	VectorXi waypoint(2);
	waypoint << current_node.x, current_node.y;
	path.push(waypoint);

	if (current_node_idx == goal_node_idx) return path;

	std::vector<RRT_Edge> edges = RapidTree.getIncidentEdges(current_node_idx);
	for (int i = 0; i < edges.size(); i++) {

		// Move up to Parent Node
		if (edges[i].child_index == current_node_idx)
			return path_trace_helper(goal_node_idx, edges[i].parent_index, path);
	}
	return path;
}




std::vector<VectorXi> RRT::path_trace(int goal_node_idx, int current_node_idx) {

	std::stack<VectorXi> path_in;
	std::stack<VectorXi> path_out = path_trace_helper(goal_node_idx, current_node_idx, path_in);
	std::vector<VectorXi> waypoints;

	int path_size = path_out.size();
	for (int i = 0; i < path_size; i++) {
		waypoints.push_back(path_out.top());
		path_out.pop();
	}

	return waypoints;
}


// Public-------------------------------------------------------------------------------------------------------------------------------------

RRT::RRT() { /*Default Constructor*/ }


RRT::RRT(Eigen::Tensor<float, 2> map) : MAP(map) { 

	auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
	// std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}



void RRT::loadMAP(Eigen::Tensor<float, 2> map) {

    MAP = map;

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}



std::vector<VectorXi> RRT::rrtPath(VectorXi start, VectorXi goal, float maxConnectionDistance) {
	
	MaxConnectionDistance = maxConnectionDistance;
	RRT_Edge null_edge;
	Goal.setPoint(goal[0], goal[1]);
	Start.setPoint(start[0], start[1]);
	Start.dist_from_start = 0.f;

	if (!is_start_and_goal_valid()) {
		VectorXi start_pt(2); start_pt << Start.x, Start.y;
		std::vector<VectorXi> invalid_path;
		invalid_path.push_back(start_pt);
		return invalid_path;
	}

    RapidTree.addVertex(Start, false, null_edge);
    int recently_added = RapidTree.getNumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (!(RapidTree.getVertex(recently_added).x == Goal.x && RapidTree.getVertex(recently_added).y == Goal.y)) {

		repeat_node = false;
        RRT_Node randP = get_random_position();
		std::pair<int, float> nearestVertex = get_nearest_vertex_index(randP);
		if (repeat_node) { continue; }
        if (!connect_new_vertex(nearestVertex, randP)) continue;
		connect_new_vertex(nearestVertex, randP);
		recently_added = RapidTree.getNumOfVertices() - 1;

		// std::cout << "Added to Tree (" << RapidTree.Get_Vertex(recently_added).x << ", " << RapidTree.Get_Vertex(recently_added).y << ")" << std::endl;
		// std::cout << "Tree Size: " << RapidTree.Get_NumOfVertices() << std::endl;
    }

	return path_trace(0, RapidTree.getNumOfVertices() - 1);
}



std::vector<VectorXi> RRT::rrtStarPath(VectorXi start, VectorXi goal, float maxConnectionDistance, float search_radius) {
	
	MaxConnectionDistance = maxConnectionDistance;
	SearchRadius = search_radius;
	RRT_Edge null_edge;
	Goal.setPoint(goal[0], goal[1]);
	Start.setPoint(start[0], start[1]);
	Start.dist_from_start = 0.f;
	
	if (!is_start_and_goal_valid()) {
		VectorXi start_pt(2); start_pt << Start.x, Start.y;
		std::vector<VectorXi> invalid_path;
		invalid_path.push_back(start_pt);
		return invalid_path;
	}

	RapidTree.addVertex(Start, false, null_edge);
    int recently_added = RapidTree.getNumOfVertices() - 1;
    
    // While the Goal has not been reached.
    while (!(RapidTree.getVertex(recently_added).x == Goal.x && RapidTree.getVertex(recently_added).y == Goal.y) ) {
		
		repeat_node = false;
		std::vector<int> neighbors;
        RRT_Node randP = get_random_position();
		std::pair<int, float> nearest_index = get_nearest_vertex_index(randP);
		if (repeat_node) continue;
		
		if (nearest_index.second <= SearchRadius) {
			if (!connect_new_vertex(nearest_index, randP)) continue;
		}

		// Re-wire other neighbors in search radius
		if (!get_neighbors(randP, SearchRadius, neighbors)) continue;
		int randP_index = RapidTree.getNumOfVertices() - 1;
		rewire_neighbors(neighbors, randP_index);

		recently_added = randP_index;

		// std::cout << "Added to Tree (" << RapidTree.Get_Vertex(recently_added).x << ", " << RapidTree.Get_Vertex(recently_added).y << ")" << std::endl;
		// std::cout << "Tree Size: " << RapidTree.Get_NumOfVertices() << std::endl;
    }

	return path_trace(0, RapidTree.getNumOfVertices() - 1);
}




/*
 * 			TO-DO
 * 			-----
 *  - 
 *
 *  - 
 *
 *  - 
 *  */
