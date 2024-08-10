#include "../include/FrontierExploration.hpp"

// NOT FINISHED!!!!!!!!!!!

// Private-----------------------------------------------------------------------------------------------------------------------------------
void FrontierExplorer::Build_CellStatusMap() {

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

void FrontierExplorer::Build_RecursionMap(RecursionPoint **&RecursionMap) {

    RecursionMap = new RecursionPoint*[M];
    for (int i = 0; i < M; i++) {
        RecursionMap[i] = new RecursionPoint[N];
    }
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {

            RecursionPoint cell = RecursionPoint(false);
            RecursionMap[i][j] = cell;
        }
    }
}

bool FrontierExplorer::isValid(int row, int col) {
    return (col >= 0) && (col < N) && 
        (row >= 0) && (row < M);
}

bool FrontierExplorer::isFrontier(VectorXf point) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
 
    // North Cell: (x-1, y)
    if (isValid(row, col - 1) && Map(row, col - 1) == 0.5) { return true; }

    // South Cell (x+1, y)
    if (isValid(row, col + 1) && Map(row, col + 1) == 0.5) { return true; }

    // East Cell (x, y+1)
    if (isValid(row + 1, col) && Map(row + 1, col) == 0.5) { return true; }

    // West Cell (x, y-1)
    if (isValid(row - 1, col) && Map(row - 1, col) == 0.5) { return true; }

    // North-East Cell (x-1, y+1)
    if (isValid(row + 1, col - 1) && Map(row + 1, col - 1) == 0.5) { return true; }

    // North-West Cell (x-1, y-1)
    if (isValid(row - 1, col - 1) && Map(row - 1, col - 1) == 0.5) { return true; }

    // South-East Cell (x+1, y+1)
    if (isValid(row + 1, col + 1) && Map(row + 1, col + 1) == 0.5) { return true; }

    // South-West Cell (x+1, y-1)
    if (isValid(row - 1, col + 1) && Map(row - 1, col + 1) == 0.5) { return true; }

    return false;

}

VectorXf FrontierExplorer::Get_Centroid(std::vector<VectorXf> frontier) {

    VectorXf centroid(2);
    float x_sum = 0;
    float y_sum = 0;
    for (int j = 0; j < frontier.size(); j++) {

        x_sum += frontier[j][0];
        y_sum += frontier[j][1];
    }

    centroid << x_sum / (frontier.size()), y_sum / (frontier.size());
    return centroid;
}

void FrontierExplorer::Get_AdjacentCells(VectorXf point, std::vector<VectorXf> &adjacents, RecursionPoint **RecursionMap) {

    adjacents.push_back(point);

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!

    RecursionMap[col][row].added = true;

    VectorXf north(2);
    VectorXf south(2);
    VectorXf east(2);
    VectorXf west(2);
    VectorXf north_east(2);
    VectorXf north_west(2);
    VectorXf south_east(2);
    VectorXf south_west(2);
    north << row, col - 1;
    south << row, col + 1;
    east << row + 1, col;
    west << row - 1, col;
    north_east << row + 1, col - 1;
    north_west << row - 1, col - 1;
    south_east << row + 1, col + 1;
    south_west << row - 1, col + 1;

    // North Cell: (x-1, y)
    if (isValid(row, col - 1) && isFrontier(north) && !RecursionMap[col - 1][row].added) { Get_AdjacentCells(north, adjacents, RecursionMap); }

    // South Cell (x+1, y)
    if (isValid(row, col + 1) && isFrontier(south) && !RecursionMap[col + 1][row].added) { Get_AdjacentCells(south, adjacents, RecursionMap); }

    // East Cell (x, y+1)
    if (isValid(row + 1, col) && isFrontier(east) && !RecursionMap[col][row + 1].added) { Get_AdjacentCells(east, adjacents, RecursionMap); }

    // West Cell (x, y-1)
    if (isValid(row - 1, col) && isFrontier(west) && !RecursionMap[col][row - 1].added) { Get_AdjacentCells(west, adjacents, RecursionMap); }

    // North-East Cell (x-1, y+1)
    if (isValid(row + 1, col - 1) && isFrontier(north_east) && !RecursionMap[col - 1][row + 1].added) { Get_AdjacentCells(north_east, adjacents, RecursionMap); }

    // North-West Cell (x-1, y-1)
    if (isValid(row - 1, col - 1) && isFrontier(north_west) && !RecursionMap[col - 1][row - 1].added) { Get_AdjacentCells(north_west, adjacents, RecursionMap); }

    // South-East Cell (x+1, y+1)
    if (isValid(row + 1, col + 1) && isFrontier(south_east) && !RecursionMap[col + 1][row + 1].added) { Get_AdjacentCells(south_east, adjacents, RecursionMap); }

    // South-West Cell (x+1, y-1)
    if (isValid(row - 1, col + 1) && isFrontier(south_west) && !RecursionMap[col + 1][row - 1].added) { Get_AdjacentCells(south_west, adjacents, RecursionMap); }

    return;

    
}


int FrontierExplorer::Get_CellStatus(VectorXf point, int map_frontier) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    

    if (map_frontier == MAP_STATUS) 
        return CellStatusMap[row][col].map_status;


    else if (map_frontier == FRONTIER_STATUS)
        return CellStatusMap[row][col].frontier_status;
}



void FrontierExplorer::Update_CellStatus(VectorXf point, int map_frontier, int status) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    

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


bool FrontierExplorer::Has_OpenSpaceNeighbor(VectorXf point) {

    int row = point[1]; // Change VectorXf to VectorXi!!!!!!!!!!!!
    int col = point[0]; // Change VectorXf to VectorXi!!!!!!!!!!!!
 
    // North Cell: (x-1, y)
    if (isValid(row, col - 1) && Map(row, col - 1) < 0.5) { return true; }

    // South Cell (x+1, y)
    if (isValid(row, col + 1) && Map(row, col + 1) < 0.5) { return true; }

    // East Cell (x, y+1)
    if (isValid(row + 1, col) && Map(row + 1, col) < 0.5) { return true; }

    // West Cell (x, y-1)
    if (isValid(row - 1, col) && Map(row - 1, col) < 0.5) { return true; }

    // North-East Cell (x-1, y+1)
    if (isValid(row + 1, col - 1) && Map(row + 1, col - 1) < 0.5) { return true; }

    // North-West Cell (x-1, y-1)
    if (isValid(row - 1, col - 1) && Map(row - 1, col - 1) < 0.5) { return true; }

    // South-East Cell (x+1, y+1)
    if (isValid(row + 1, col + 1) && Map(row + 1, col + 1) < 0.5) { return true; }

    // South-West Cell (x+1, y-1)
    if (isValid(row - 1, col + 1) && Map(row - 1, col + 1) < 0.5) { return true; }

    return false;
}


std::vector<std::vector<VectorXf>> FrontierExplorer::Detect_WavefrontFrontier(VectorXf robot_pose) {

    MapQueue.push(robot_pose);
    MapOpenList.push_back(robot_pose);
    std::vector<std::vector<VectorXf>> frontiers;
    Update_CellStatus(robot_pose, MAP_STATUS, xOPEN);


    while (!MapQueue.empty()) {

        VectorXf point = MapQueue.front();
        MapQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED) { continue; }

        if (isFrontier(point)) {
            std::vector<VectorXf> new_frontier = Extract_Frontier2D(point);
            frontiers.push_back(new_frontier);
        }

        RecursionPoint **recursion_map;
        Build_RecursionMap(recursion_map);
        std::vector<VectorXf> neighbors;
        Get_AdjacentCells(point, neighbors, recursion_map);

        for (int i = 0; i < neighbors.size(); i++) {

            if (Get_CellStatus(point, MAP_STATUS) == xNONE && Has_OpenSpaceNeighbor(neighbors[i])) {
                MapQueue.push(neighbors[i]);
                Update_CellStatus(neighbors[i], MAP_STATUS, xOPEN);
            }

        }
        Update_CellStatus(point, MAP_STATUS, xCLOSED);
    }

    return frontiers;
}

std::vector<VectorXf> FrontierExplorer::Extract_Frontier2D(VectorXf frontier_pt) {

    FrontierQueue.push(frontier_pt);
    FrontierOpenList.push_back(frontier_pt);
    std::vector<VectorXf> new_frontier;

    while (!FrontierQueue.empty()) {

        VectorXf point = FrontierQueue.front();
        FrontierQueue.pop();

        if (Get_CellStatus(point, MAP_STATUS) == xCLOSED || Get_CellStatus(point, FRONTIER_STATUS) == xCLOSED) { continue; }

        if (isFrontier(point)) {
            
            new_frontier.push_back(point);
            RecursionPoint **recursion_map;
            Build_RecursionMap(recursion_map);
            std::vector<VectorXf> neighbors;
            Get_AdjacentCells(point, neighbors, recursion_map);

            for (int i = 0; i < neighbors.size(); i++) {

                if (Get_CellStatus(point, FRONTIER_STATUS) == xNONE && Get_CellStatus(point, MAP_STATUS) != xCLOSED) {
                    MapQueue.push(neighbors[i]);
                    Update_CellStatus(neighbors[i], FRONTIER_STATUS, xOPEN);
                }
            }
        }

        Update_CellStatus(point, FRONTIER_STATUS, xCLOSED);
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

    Build_CellStatusMap();
}


void FrontierExplorer::Load_MAP(Eigen::Tensor<float, 2> map) {

    Map = map;
    auto &d = Map.dimensions();
    M = d[0];
	N = d[1];

    Build_CellStatusMap();
}


VectorXf FrontierExplorer::FindFrontier(VectorXf robot_pose) {

    std::vector<std::vector<VectorXf>> frontiers = Detect_WavefrontFrontier(robot_pose);
    
    // Decide on frontier to visit----------
    // TODO: Currently only takes into account closest frontier, not mix between closest and largest
    VectorXf closest_centroid(2);
    float closest_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < frontiers.size(); i++) {

        VectorXf centroid = Get_Centroid(frontiers[i]);
        float dist = std::sqrt(std::pow((centroid[0] - robot_pose[0]), 2) + std::pow((centroid[0] - robot_pose[0]), 2));
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }
    return closest_centroid;
}



