#include "../include/FrontierExploration.hpp"

// Private-----------------------------------------------------------------------------------------------------------------------------------
void FrontierExplorer::build_cell_status_map() {

    CellStatusMap = new PointStatus*[M];
    for (int i = 0; i < M; i++) {
        CellStatusMap[i] = new PointStatus[N];
    }
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {

            MapStatus map_status;
            FrontierStatus frontier_status;
            map_status = M_NONE;
            frontier_status = F_NONE;
            PointStatus cell = PointStatus(map_status, frontier_status);
            CellStatusMap[i][j] = cell;
        }
    }
}

bool FrontierExplorer::is_valid(int row, int col) {
    return (col >= 0) && (col < N) && 
        (row >= 0) && (row < M);
}

bool FrontierExplorer::is_frontier_point(VectorXi point) {

    // If Point is not an unknown space it can't be a frontier
    if (Map(point[1], point[0]) != 0.5) { return false; }

    return has_open_space_neighbor(point);
}


bool FrontierExplorer::has_open_space_neighbor(VectorXi point) {

    int row = point[1];
    int col = point[0];

    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

    for (auto& dir : directions) {
        int ncol = col + dir[0];
        int nrow = row + dir[1];

        if (is_valid(nrow, ncol) && Map(nrow, ncol) < 0.5) { return true; }
    }

    return false;
}


VectorXi FrontierExplorer::get_centroid(std::vector<VectorXi> frontier) {

    VectorXi centroid(2);
    float x_sum = 0;
    float y_sum = 0;
    for (int j = 0; j < frontier.size(); j++) {

        x_sum += frontier[j][0];
        y_sum += frontier[j][1];
    }

    centroid << x_sum / (frontier.size()), y_sum / (frontier.size());
    return centroid;
}

int FrontierExplorer::get_cell_status(VectorXi point, int map_frontier) {

    int row = point[1];
    int col = point[0];

    if (!is_valid(row, col)) {
        std::cerr << "ERROR: Cell (" << row << ", "<< col << ") Out of Bounds [Cannot Find Frontier]" << std::endl;
    }
    
    if (map_frontier == MAP_STATUS) 
        return CellStatusMap[row][col].map_status;


    else if (map_frontier == FRONTIER_STATUS)
        return CellStatusMap[row][col].frontier_status;
}



void FrontierExplorer::update_cell_status(VectorXi point, int map_frontier, int status) {

    int row = point[1];
    int col = point[0];

    if (!is_valid(row, col)) {
        std::cerr << "ERROR: Cell (" << row << ", "<< col << ") Out of Bounds [Cannot Find Frontier]" << std::endl;
        return;
    }
    
    if (map_frontier == MAP_STATUS) {
        
        if (status == xOPEN) 
            CellStatusMap[row][col].map_status = M_OPEN;

        else if (status == xCLOSED) 
            CellStatusMap[row][col].map_status = M_CLOSED;
        
    }

    else if (map_frontier == FRONTIER_STATUS) {

        if (status == xOPEN) 
            CellStatusMap[row][col].frontier_status = F_OPEN;

        else if (status == xCLOSED) 
            CellStatusMap[row][col].frontier_status = F_CLOSED;
    }
}




std::vector<std::vector<VectorXi>> FrontierExplorer::detect_wavefront_frontier(VectorXi robot_index) {

    MapQueue.push(robot_index);
    MapOpenList.push_back(robot_index);
    std::vector<std::vector<VectorXi>> frontiers;
    update_cell_status(robot_index, MAP_STATUS, xOPEN);


    while (!MapQueue.empty()) {

        VectorXi point = MapQueue.front();
        MapQueue.pop();
        // std::cout << "Viewing Point: " << point.transpose() << std::endl;

        if (get_cell_status(point, MAP_STATUS) == xCLOSED) { continue; }

        if (is_frontier_point(point)) {
            std::vector<VectorXi> new_frontier = extract_frontier_2D(point);
            // std::cout << "Frontier of SIZE " << new_frontier.size() << " Created." << std::endl;
            frontiers.push_back(new_frontier);
            // std::cout << "NEW FRONTIER POINT FOUND!!" << std::endl; 
        }

        // Direction vectors for 8-connected neighbors
        int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

        // Check all 8-connected neighbors
        for (auto& dir : directions) {
            VectorXi neighbor(2);
            int neighborx = point[0] + dir[0];
            int neighbory = point[1] + dir[1];
            neighbor << neighborx, neighbory;

            // Check bounds
            if (!is_valid(neighbory, neighborx)) {
                continue;
            }

            //NOTE: 'Has_OpenSpaceNeighbor(neighbor)' Prevents you from propagating deep into unknown space.
            if (get_cell_status(neighbor, MAP_STATUS) == xNONE && has_open_space_neighbor(neighbor)) {
                update_cell_status(neighbor, MAP_STATUS, xOPEN);
                MapQueue.push(neighbor);
                // std::cout << "Pushing Neighbor: " << neighbor.transpose() << std::endl;
            }
        }
        update_cell_status(point, MAP_STATUS, xCLOSED);
    }

    return frontiers;
}

std::vector<VectorXi> FrontierExplorer::extract_frontier_2D(VectorXi frontier_pt) {

    FrontierQueue.push(frontier_pt);
    FrontierOpenList.push_back(frontier_pt);
    std::vector<VectorXi> new_frontier;

    while (!FrontierQueue.empty()) {

        VectorXi point = FrontierQueue.front();
        FrontierQueue.pop();

        if (get_cell_status(point, MAP_STATUS) == xCLOSED || get_cell_status(point, FRONTIER_STATUS) == xCLOSED) { continue; }

        if (is_frontier_point(point)) {
            
            new_frontier.push_back(point);

            // Direction vectors for 8-connected neighbors
            int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                                {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

             // Check all 8-connected neighbors
            for (auto& dir : directions) {
                VectorXi neighbor(2);
                int neighborx = point[0] + dir[0];
                int neighbory = point[1] + dir[1];
                neighbor << neighborx, neighbory;

                // Check bounds
                if (!is_valid(neighbory, neighborx)) {
                    continue;
                }

                if (get_cell_status(neighbor, FRONTIER_STATUS) == xNONE && get_cell_status(neighbor, MAP_STATUS) != xCLOSED) {
                    update_cell_status(neighbor, FRONTIER_STATUS, xOPEN);
                    FrontierQueue.push(neighbor);
                }
            }
        }

        update_cell_status(point, FRONTIER_STATUS, xCLOSED);
    }

    return new_frontier;
}

// TODO: Will likely need to add code where you update list status to M_NONE or F_NONE ?????

// Public-----------------------------------------------------------------------------------------------------------------------------------
FrontierExplorer::FrontierExplorer() { }

FrontierExplorer::FrontierExplorer(Eigen::Tensor<float, 2> map) : Map(map) { 

    auto &d = Map.dimensions();
    M = d[0];
	N = d[1];
    build_cell_status_map();
}


void FrontierExplorer::loadMAP(Eigen::Tensor<float, 2> map) {

    Map = map;
    auto &d = Map.dimensions();
    M = d[0];
	N = d[1];
    build_cell_status_map();
}


VectorXi FrontierExplorer::findFrontier(VectorXi robot_pose) {

    std::vector<std::vector<VectorXi>> frontiers = detect_wavefront_frontier(robot_pose);
    // std::cout << Map << std::endl;
    
    // Decide on frontier to visit----------
    // TODO: Currently only takes into account closest frontier, not mix between closest and largest
    VectorXi closest_centroid = robot_pose;
    float closest_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < frontiers.size(); i++) {

        VectorXi centroid = get_centroid(frontiers[i]);
        
        float dist = std::hypot((centroid[0] - robot_pose[0]), (centroid[1] - robot_pose[1]));
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }

    std::cout << "Start: " << robot_pose.transpose() << " -----------> Closest Frontier!: " << closest_centroid.transpose() 
        << "     [# of Frontiers Found: " << frontiers.size() << "]" << std::endl;
    return closest_centroid;
}



