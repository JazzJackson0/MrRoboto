#include "../include/kd_tree.hpp"

// Private ------------------------------------------------------------------------
int KDTree::get_depth(Node* node) {
    if (!node) return 0;
    return 1 + std::max(get_depth(node->left), get_depth(node->right));
}

void KDTree::collect_points(Node* node, std::vector<VectorXf> &points) {
    if (!node) return;
    points.push_back(node->packet.data);
    collect_points(node->left, points);
    collect_points(node->right, points);
}

Node* KDTree::build_balanced_tree(std::vector<VectorXf> &points, int depth) {
    
    if (points.empty()) return nullptr;

    // 1. Find the median of the points along the "current" splitting axis (depth % k).
    int axis = depth % k;
    std::sort(points.begin(), points.end(), [axis](const VectorXf &a, const VectorXf &b) {
        return a[axis] < b[axis];
    });
    int median_idx = points.size() / 2;

    // 2. Use the median as the new root for the subtree.
    Node *newNode = new Node;
    newNode->packet.data = points[median_idx];
    // std::cout << "NODE REBALANCE: " << newNode->packet.data.transpose() << std::endl;

    std::vector<VectorXf> left_points(points.begin(), points.begin() + median_idx);
    std::vector<VectorXf> right_points(points.begin() + median_idx + 1, points.end());

    // 3. Recursively apply steps 1-2 for the left and right subtrees.
    newNode->left = build_balanced_tree(left_points, depth + 1);
    newNode->right = build_balanced_tree(right_points, depth + 1);

    return newNode;
}

float KDTree::get_radius_squared(VectorXf point_a, VectorXf point_b) {
    
    // Compute sum of squared differences in single step using optimized SIMD instructions.
    // x^2 - y^2 - ...
    return (point_a - point_b).squaredNorm();
}

Node * KDTree::get_closest(VectorXf point, Node *node_a, Node *node_b) {

    if (node_a == nullptr) return node_b;
    if (node_b == nullptr) return node_a;
    
    float node_a_rad = get_radius_squared(point, node_a->packet.data);
    float node_b_rad = get_radius_squared(point, node_b->packet.data);

    return (node_a_rad < node_b_rad)? node_a : node_b;
}

Node* KDTree::find_min(Node *current_node) {
    // TODO: Finish
    if (current_node->left == nullptr) return current_node;
    return find_min(current_node->left);
}

bool KDTree::point_match(VectorXf point_a, VectorXf point_b) {
    // Compare all elements at once and return true if they are equal.
    return (point_a.array() == point_b.array()).all();
}

Node* KDTree::insert_core(VectorXf point, Node *current_node, int depth) {

    if (current_node == nullptr) {
        current_node = new Node;
        current_node->packet.data = point;
        current_node->left = nullptr;
        current_node->right = nullptr;
        return current_node;
    }

    // Go Left
    if (point[depth % k] < current_node->packet.data[depth % k])
        current_node->left = insert_core(point, current_node->left, depth + 1);

    // Go Right
    else if (point[depth % k] >= current_node->packet.data[depth % k]) 
        current_node->right = insert_core(point, current_node->right, depth + 1);  

    return current_node;
}

// TODO: Edit this. Needs a working findMin
Node* KDTree::remove_core(VectorXf point, Node *current_node, int depth) {

    if (current_node == nullptr) return nullptr;

    // Match
    else if (point_match(point, current_node->packet.data)) {
        
        // If leaf node
        if (current_node->left == nullptr && current_node->right == nullptr) {
            delete current_node;
            return nullptr;
        }

        else if (current_node->right != nullptr) {  
            // Replace node to delete with min
            Node *min = find_min(current_node->right);
            current_node->packet.data = min->packet.data;

            // Delete the Min
            current_node->right = remove_core(min->packet.data, current_node->right, depth + 1);
        }

        else if (current_node->left != nullptr) {
            // Replace node to delete with min
            Node *min = find_min(current_node->left);
            current_node->packet.data = min->packet.data;
            
            // Delete the Min
            current_node->right = remove_core(min->packet.data, current_node->left, depth + 1);
        }

        return current_node;   
    }

    // Go Left
    else if (point[depth % k] < current_node->packet.data[depth % k]) {
        
        current_node->left = remove_core(point, current_node->left, depth + 1); 
    }

    // Go Right
    else if (point[depth % k] >= current_node->packet.data[depth % k]) {

        current_node->right = remove_core(point, current_node->right, depth + 1);  
    }

    return current_node;
}


Node* KDTree::search_core(VectorXf point, Node *current_node, int depth) {

    Node *found = NULL;

    if (current_node == nullptr) return NULL;

    // Match
    else if (point[depth % k] == current_node->packet.data[depth % k]) {

        if (point_match(point, current_node->packet.data)) 
            found = current_node;
        
        else 
            found = search_core(point, current_node->right, depth + 1);     
    }

    // Go Left
    else if (point[depth % k] < current_node->packet.data[depth % k]) 
        found = search_core(point, current_node->left, depth + 1);
    

    // Go Right
    else if (point[depth % k] > current_node->packet.data[depth % k]) 
        found = search_core(point, current_node->right, depth + 1);
    
    return found;
}


Node* KDTree::get_nearest_neighbor(VectorXf point, Node *current_node, int depth) {

    Node *nextBranch = NULL;
    Node *otherBranch = NULL;

    if (current_node == nullptr) return nullptr; 

    // Prepare to Go Left
    if (point[depth % k] < current_node->packet.data[depth % k]) {
        nextBranch = current_node->left;
        otherBranch = current_node->right;
    }

    // Prepare to Go Right
    else {
        nextBranch = current_node->right;
        otherBranch = current_node->left;
    }

    // Continue Search
    Node *closest_in_subtree = get_nearest_neighbor(point, nextBranch, depth + 1);
    Node *current_nearest = get_closest(point, current_node, closest_in_subtree);

    // Get Stats for Nearest
    float radius_squared = get_radius_squared(point, current_nearest->packet.data);
    float dist = point[depth % k] - current_node->packet.data[depth % k];

    // Go into other subtree if distance to it is less than distance to current nearest neighbor
    if ((dist * dist) < radius_squared) {
        closest_in_subtree = get_nearest_neighbor(point, otherBranch, depth + 1);
        current_nearest = get_closest(point, current_node, closest_in_subtree);
    } 

    return current_nearest;
}


// Public -------------------------------------------------------------------------
KDTree::KDTree() {
    kd_tree = nullptr;
}


KDTree::KDTree(int k) : KDTree() {

    this->k = k;
}

Node* KDTree::getKDTree() {
    return kd_tree;
}

void KDTree::insert(VectorXf point) {
    VectorXf pt = point.head<2>();
    kd_tree = insert_core(pt, kd_tree, 0);
}


void KDTree::remove(VectorXf point) {
    VectorXf pt = point.head<2>();
    kd_tree = remove_core(pt, kd_tree, 0);
}


Packet KDTree::search(VectorXf point) {
    VectorXf pt = point.head<2>();
    return search_core(pt, kd_tree, 0)->packet;
}

Packet KDTree::nearestNeighbor(VectorXf point) {
    VectorXf pt = point.head<2>();
    return get_nearest_neighbor(pt, kd_tree, 0)->packet;
}


void KDTree::addData(VectorXf point, float weight) {
    VectorXf pt = point.head<2>();
    Packet pckt;
    pckt.data = pt;
    pckt.weight = weight;
    raw_points.push_back(pckt);

    // Maintain sorted order (sorted by x position)
    for (int i = raw_points.size() - 1; i > 0; i--) {
        if (raw_points[i].data[0] < raw_points[i - 1].data[0]) {
            VectorXf temp = raw_points[i - 1].data;
            raw_points[i - 1].data = raw_points[i].data;
            raw_points[i].data = temp;
        }
    }
}


void KDTree::buildKDTree() {

    if (raw_points.empty()) {
        std::cerr << "ERROR: No data available to build tree" << std::endl;
        return;
    }
    
    int median_idx = raw_points.size() / 2;
    VectorXf median = raw_points[median_idx].data;
    kd_tree = insert_core(median, kd_tree, 0);

    for (int i = 0; i < median_idx; i++) {
        // std::cout << "Inserting: (" << raw_points[i][0] << ", " << raw_points[i][1] << ")" << std::endl;
        kd_tree = insert_core(raw_points[i].data, kd_tree, 0);
    }
    for (int i = median_idx + 1; i < raw_points.size(); i++) {
        // std::cout << "Inserting: (" << raw_points[i][0] << ", " << raw_points[i][1] << ")" << std::endl;
        kd_tree = insert_core(raw_points[i].data, kd_tree, 0);
    }

    int leftDepth = get_depth(kd_tree->left);
    int rightDepth = get_depth(kd_tree->right);
    if (abs(leftDepth - rightDepth) > 25) {  // Balance threshold (allowable depth difference)
        // std::cout << "REEEEEEBBBBAAAALLLLLLLAAAAAANNNNNNCCCCCCEEEEEEE!!!@@@@!!!@@@@!!!@@@@!!!@@@@!!!@@@@!!!@@@@!!!@@@@!!!@@@@" << std::endl;
        std::vector<VectorXf> points;
        collect_points(kd_tree, points);
        kd_tree = build_balanced_tree(points, 0);
    }
}












