#include "../include/AStar.hpp"

float AStar::get_manhattan_distance(int x1, int y1, int x2, int y2) {

    // Returns H-Cost
    return (float)abs(x1 - x2) + (float)abs(y1 - y2);
}



float AStar::get_diagonal_distance(int x1, int y1, int x2, int y2) {

    float nodeDistance = 1;
    float nodeDiagonalDistance = sqrt(2);
    float dx = (float)abs(x1 - x2);
    float dy = (float)abs(y1 - y2);
    
    // Returns H-Cost
    return (float)(nodeDistance * (dx + dy) + (nodeDiagonalDistance - (2 * nodeDistance)) * 
        (float)fmin(dx, dy));
}


float AStar::get_euclidean_distance(int x1, int y1, int x2, int y2) {

    // Returns H-Cost
    return std::hypot(( x1 - x2 ), ( y1 - y2 ));
}


bool AStar::is_valid(int x, int y) {
    return (x >= 0) && (x < WIDTH) && 
        (y >= 0) && (y < HEIGHT);
}



bool AStar::is_blocked(int x, int y) {

    int row = y;
    int col = x;
    return MAP(row, col) > 0.5f;
}


bool AStar::is_goal_reached(int x, int y) {

    return ( x == Goal[0] && y == Goal[1]);
}


bool AStar::is_start_and_goal_valid() {
    
    if (!is_valid(Start[0], Start[1])) {
        std::cerr << "ERROR: Invalid Starting Coordinates." << std::endl;
        return false;
    }

    if (!is_valid(Goal[0], Goal[1])) {
        std::cerr << "ERROR: Invalid Goal Coordinates." << std::endl;
        return false;
    }

    if (is_blocked(Start[0], Start[1])) {
        std::cerr << "ERROR: Start Coodrdinates Blocked." << std::endl;
        return false;
    }

    if (is_goal_reached(Start[0], Start[1])) {
        std::cerr << "ERROR: Ummm... You're already there..." << std::endl;
        return false;
    }

    return true;
}



float AStar::get_h_cost(int x, int y, DistanceFormula formula) {

    int goal_x = Goal[0];
    int goal_y = Goal[1];

    if (formula == MANHATTAN) { return get_manhattan_distance(x, y, goal_x, goal_y); }

    if (formula == DIAGONAL) { return get_diagonal_distance(x, y, goal_x, goal_y); }

    if (formula == EUCLID) { return get_euclidean_distance(x, y, goal_x, goal_y); }

    return 0.0;
}


float AStar::get_g_cost(int x1, int y1, int x2, int y2, DistanceFormula formula) {

    if (formula == MANHATTAN) { return get_manhattan_distance(x1, y1, x2, y2); }

    if (formula == DIAGONAL) { return get_diagonal_distance(x1, y1, x2, y2); }

    if (formula == EUCLID) { return get_euclidean_distance(x1, y1, x2, y2); }

    return 0.0;
}


void AStar::get_adjacent_cell_coordinates(int x, int y, int &x_adjacent, int &y_adjacent, int adjacent_cell_num) {

    switch (adjacent_cell_num) {
        case 0: // North Cell: (x-1, y)
            x_adjacent = x - 1;
            y_adjacent = y;
            break;
        case 1: // South Cell (x+1, y)
            x_adjacent = x + 1;
            y_adjacent = y;
            break;
        case 2: // East Cell (x, y+1)
            x_adjacent = x;
            y_adjacent = y + 1;
            break;
        case 3: // West Cell (x, y-1)
            x_adjacent = x;
            y_adjacent = y - 1;
            break;
        case 4: // North-East Cell (x-1, y+1)
            x_adjacent = x - 1;
            y_adjacent = y + 1;
            break;
        case 5: // North-West Cell (x-1, y-1)
            x_adjacent = x - 1;
            y_adjacent = y - 1;
            break;
        case 6: // South-East Cell (x+1, y+1)
            x_adjacent = x + 1;
            y_adjacent = y + 1;
            break;
        case 7: // South-West Cell (x+1, y-1)
            x_adjacent = x + 1;
            y_adjacent = y - 1;
            break;
    }
}



std::vector<std::vector<astar::Cell>> AStar::init_matrix_of_cells() {

    astar::Cell default_cell;
    default_cell.FCost = FLT_MAX - 1;
    default_cell.GCost = 0.f;
    default_cell.HCost = FLT_MAX - 1;
    default_cell.parentY = -1;
    default_cell.parentX = -1;
    std::vector<std::vector<astar::Cell>> cells(WIDTH, std::vector<astar::Cell>(HEIGHT, default_cell));

    // Inititalize the 'Start' Cell
    int x = Start[0];
    int y = Start[1];

    cells[x][y].FCost = 0.0;
    cells[x][y].GCost = 0.0;
    cells[x][y].HCost = 0.0;
    cells[x][y].parentY = x;
    cells[x][y].parentX = y;

    return cells;
}

void AStar::path_trace(std::vector<std::vector<astar::Cell>> cells) {
    
    int x = Goal[0];
    int y = Goal[1];

    stack<VectorXi> Path;

    // Follow the trail of Visited cells. From GOAL to START
    // While Loop End Contition: When cell's current coordinates are the same as it's parent coordinaates, you have reached the Starting cell.
    while (!(cells[x][y].parentX == x && cells[x][y].parentY == y)) {
        
        //std::cout << "Cell: [" << x << ", " << y << "]" << std::endl;

        // Push cell onto stack, then move to its parent
        VectorXi cell(2);
        cell << x, y;
        Path.push(cell);
        int tempX = cells[x][y].parentX;
        int tempY = cells[x][y].parentY;
        x = tempX;
        y = tempY;
        
        //std::cout << "Parent: [" << x << ", " << y << "]" << std::endl;
        //std::cout << std::endl;
    }
    
    // Push Start cell onto stack
    VectorXi start_cell(2);
    start_cell << x, y;
    Path.push(start_cell);

    // Create the Path vector From START to GOAL
    while ( !(Path.empty()) ) {
        VectorXi cell = Path.top();
        ThePath.push_back(cell);
        Path.pop();
    }

    return;
}

AStar::AStar() {
    // Defualt Constructor
}

AStar::AStar(Eigen::Tensor<float, 2> map) : MAP(map) {

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}


void AStar::loadMAP(Eigen::Tensor<float, 2> map) {

    MAP = map;

    auto &d = MAP.dimensions();
	WIDTH = d[1];
	HEIGHT = d[0];
    // std::cout << std::endl;
    // std::cout << "Map:" << std::endl;
    // std::cout << MAP << std::endl;
    // std::cout << std::endl;
}


std::vector<VectorXi> AStar::path(VectorXi startCell, VectorXi goalCell) {

    ThePath.clear();
    Start = startCell;
    Goal = goalCell;
    if (!is_start_and_goal_valid()) { 
        ThePath.push_back(startCell);
        std::cerr << "ERROR: Invalid Coordinates for START (" << startCell.transpose() << ") or GOAL (" << goalCell.transpose() << ")" << std::endl;
        return ThePath; 
    }

    std::vector<std::vector<bool>> Visited(WIDTH, std::vector<bool>(HEIGHT, false));
    auto cmp = [](astar::Cell cell_a, astar::Cell cell_b) {
        if (cell_a.FCost == cell_b.FCost) {
            return cell_a.HCost > cell_b.HCost;
        }
        return cell_a.FCost > cell_b.FCost;
    };
    std::priority_queue<astar::Cell, std::vector<astar::Cell>, decltype(cmp)> Uncovered(cmp);
    std::vector<std::vector<astar::Cell>> MapOfCells = init_matrix_of_cells();
    
    // Mark the starting node as "Uncovered" and "Visited"
    astar::Cell start;
    start.x = Start[0];
    start.y = Start[1];
    start.parentX = Start[0];
    start.parentY = Start[1];
    start.FCost = 0.f;
    MapOfCells[start.x][start.y] = start;
    Uncovered.push(start);
    
    while (!Uncovered.empty()) {

        float New_FCost, New_GCost, New_HCost;

        // Visit lowest cost node
        //std::cout << "To Visit Size: " << Uncovered.size() << std::endl;
        astar::Cell cell = Uncovered.top();
        Uncovered.pop();
        //std::cout << "Lowest Cost: (" << cell.x << ", " << cell.y << ")" << std::endl; 
        
        Visited[cell.x][cell.y] = true;
        // std::cout << "(" << cell.x << ", " << cell.y << ")" << std::endl;

        // Look at the 8 Adjacent Cells
        int x_adjacent, y_adjacent;
        for (int i = 0; i < 8; i++) {

            get_adjacent_cell_coordinates(cell.x, cell.y, x_adjacent, y_adjacent, i);

            if (!is_valid(x_adjacent, y_adjacent) || is_blocked(x_adjacent, y_adjacent) || (Visited[x_adjacent][y_adjacent])) { continue; }

            // If the Goal has been reached
            if (is_goal_reached(x_adjacent, y_adjacent)) {

                MapOfCells[x_adjacent][y_adjacent].parentX = cell.x;
                MapOfCells[x_adjacent][y_adjacent].parentY = cell.y;
                path_trace(MapOfCells);

                return ThePath;
            }

            // Re-Calculate Costs
            New_GCost = get_g_cost(x_adjacent, y_adjacent, cell.x, cell.y, EUCLID) + MapOfCells[cell.x][cell.y].GCost;
            New_HCost = get_h_cost(x_adjacent, y_adjacent, EUCLID);
            New_FCost = New_GCost + New_HCost;

            // If New F-Cost is Smaller than Old F-Cost 
            if (New_FCost < MapOfCells[x_adjacent][y_adjacent].FCost) {

                // Update Adjacent Cell and add it to 'Uncovered' list
                MapOfCells[x_adjacent][y_adjacent].FCost = New_FCost;
                MapOfCells[x_adjacent][y_adjacent].GCost = New_GCost;
                MapOfCells[x_adjacent][y_adjacent].HCost = New_HCost;
                MapOfCells[x_adjacent][y_adjacent].x = x_adjacent;
                MapOfCells[x_adjacent][y_adjacent].y = y_adjacent;
                MapOfCells[x_adjacent][y_adjacent].parentX = cell.x;
                MapOfCells[x_adjacent][y_adjacent].parentY = cell.y;

                astar::Cell cell_to_visit = MapOfCells[x_adjacent][y_adjacent];
                Uncovered.push(cell_to_visit);  

                // Test
                //std::cout << "Adding Adjacent For Future Visit: (" << x_adjacent << ", " << y_adjacent << ")" << std::endl;    
            }
        } 
    } 

    std::cerr << "ERROR: No viable A* path found." << std::endl;
    ThePath.push_back(startCell);
    return ThePath;
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







