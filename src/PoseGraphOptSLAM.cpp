#include "../include/PoseGraphOptSLAM.hpp"

void PoseGraphOptSLAM::propagate_free_space() {

	VectorXi robot_index = map_builder.mapCoordinateToDataStructureIndex(PreviousPose.pose.head<2>());
    int x_robot = robot_index[0];
    int y_robot = robot_index[1];

    // Direction vectors for 8-connected neighbors
    int directions[8][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                             {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };

    std::queue<VectorXi> q;
    q.push(robot_index);

    while (!q.empty()) {
        VectorXi index = q.front();
		int x = index[0];
		int y = index[1];
        q.pop();

        // Check all 8-connected neighbors
        for (auto& dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];

            // Check bounds
            if (nx < 0 || nx >= map_width || ny < 0 || ny >= map_height)
                continue;

            // Check if within the robot's view range
            if (std::hypot((nx - x_robot), (ny - y_robot)) < VIEW_RANGE) {

                if (map_structure_mask(ny, nx) == 0.5 && is_visible(nx, ny, x_robot, y_robot)) {
                    map_structure_mask(ny, nx) = 0.0;
					if (map_structure(ny, nx) == 0.5) map_structure(ny, nx) = 0.0;
					VectorXi new_index(2);
					new_index << nx, ny;
                    q.push(new_index);
                }
            }
        }
    }
}

bool PoseGraphOptSLAM::is_visible(int x, int y, int x_robot, int y_robot) {
    
	if (map_structure_mask(y, x) == 1.0) return false;

    // Bresenham's line algorithm for line of sight checking
    int dx = std::abs(x - x_robot);
    int dy = std::abs(y - y_robot);
    int sx = (x_robot < x) ? 1 : -1;
    int sy = (y_robot < y) ? 1 : -1;
    int err = dx - dy;

    int x_curr = x_robot;
    int y_curr = y_robot;

    while (true) {
		// Clear line of sight
        if (x_curr == x && y_curr == y) return true;

		// Line of sight blocked by obstacle
        if (map_structure_mask(y_curr, x_curr) == 1.0) return false;

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


Eigen::Tensor<float, 2> PoseGraphOptSLAM::update_map() {
	// Reset Map Mask to Default
	map_structure_mask.setConstant(0.5);

	VectorXi robot_index = map_builder.mapCoordinateToDataStructureIndex(PreviousPose.pose.head<2>());
	// VectorXf point;
	// VectorXi beam_index;
	
	Pose_Graph.iteratorStart();
	while (Pose_Graph.iteratorHasNext()) {

		Pose pose = Pose_Graph.iteratorGetData();
		#pragma omp parallel for
		for (int i = 0; i < pose.Landmarks.points.size(); i++) {
			
			VectorXf point = pose.Landmarks.points[i];
			VectorXi beam_index = map_builder.mapCoordinateToDataStructureIndex(point.head<2>());
			
			// If Beam is within Map range
			if ((beam_index[0] >= 0 && beam_index[0] < map_width) && (beam_index[1] >= 0 && beam_index[1] < map_height)) {
				#pragma omp critical 
				{
					map_structure_mask(beam_index[1], beam_index[0]) = 1.f;
					map_structure(beam_index[1], beam_index[0]) = 1.f;
				}
			}	
		}
		Pose_Graph.iteratorNext();
	}
	propagate_free_space();
	// std::cout << map_structure << std::endl;
	return map_structure;
}

float PoseGraphOptSLAM::calculate_overlap(PointCloud cloud_a, PointCloud cloud_b) {

	return (float) std::abs(std::hypot((cloud_a.mean_x - cloud_b.mean_x), (cloud_a.mean_y - cloud_b.mean_y)));
}


MatrixXf PoseGraphOptSLAM::vector_to_transformation_matrix(int x, int y, AngleAndAxis angle_axis) {

	MatrixXf R = utils::angleTo3DRotationMatrix(angle_axis);
	VectorXf t(3);
	t << x, y, 1;
	MatrixXf transformation = MatrixXf::Zero(3, 3);
	transformation.topLeftCorner(2, 2) = R;
    transformation.topRightCorner(3, 1) = t;

	return transformation;
}


void PoseGraphOptSLAM::update_state_vector() {

	rotation_axes.clear();
	CurrentPoses_n = Pose_Graph.getNumOfVertices();
	StateVector.resize(CurrentPoses_n * PoseDimensions);
	int stateVecIndex = 0;

	// Loop through each pose in graph and add it to the State Vector
	for (int i = 0; i < CurrentPoses_n; i++) {
		
		Pose p = Pose_Graph.getVertex(i);
		MatrixXf rotation = p.TransformationMatrix.block(0, 0, 3, 3);
		VectorXf translation = p.TransformationMatrix.block(0, 2, 3, 1);
		AngleAndAxis angle_axis = utils::rotationMatrix3DToAngle(rotation); // Angle & Axis of Rotation
		
		// Add pose data to State Vector 
		for (int j = 0; j < 2; j++) {
			StateVector[stateVecIndex] = translation(j);
			stateVecIndex++;
		}
		
		StateVector[stateVecIndex] = angle_axis.first;
		stateVecIndex++;
		rotation_axes.push_back(angle_axis.second);
	}

	return;
}


void PoseGraphOptSLAM::add_pose_to_graph(Pose pose, PoseEdge edge) {

	bool connected = true;

	//  If initial pose, graph starts as unconnected
	if (Pose_Graph.getNumOfVertices() == 0)
		connected = false;
		
	// Else create new edge
	else {
		// Get Relative Transformation between 2 poses
		edge.TransformationMatrix = pose.TransformationMatrix.inverse() * PreviousPose.TransformationMatrix;
		DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> covariance(3);
		covariance.diagonal().setConstant(0.01);
		edge.NoiseInfoMatrix = MatrixXf(covariance).inverse();
	}

	// Add Pose to Graph
	PreviousPose = Pose_Graph.addVertex(pose, connected, edge);
}


bool PoseGraphOptSLAM::check_for_loop_closure(Pose pose) {

	// Search Graph in given radius to find possible loop closure (Excluding the n most recently added poses)
	int closest_vertex_idx = -1;
	Pose p;
	float dist;
	
	// No Loop Closure Happening
	if (Pose_Graph.getNumOfVertices() <= NRecentPoses) return false;

	// Loop Closure Process Start-------------------------------------------------------------------------------
	float min_dist = std::numeric_limits<float>::max();
	for (int i = 0; i < Pose_Graph.getNumOfVertices() - NRecentPoses; i++) {

		p = Pose_Graph.getVertex(i);
		
		dist = std::hypot((p.pose[0] - PreviousPose.pose[0]), (p.pose[1] - PreviousPose.pose[1]));
		
		// If node is within valid loop closure Radius: Track the closest of all nodes in range.
		if (dist < ClosureDistance && (dist < min_dist)) {
			min_dist = dist;
			closest_vertex_idx = i;
		}
	}

	// Loop Closure
	if (closest_vertex_idx >= 0) {

		PoseEdge closure_edge;
		closure_edge.TransformationMatrix = pose.TransformationMatrix * 
			Pose_Graph.getVertex(closest_vertex_idx).TransformationMatrix;
		DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> covariance(3);
		covariance.diagonal().setConstant(0.01);
		closure_edge.NoiseInfoMatrix = MatrixXf(covariance).inverse();
		Pose_Graph.addEdge(Pose_Graph.getNumOfVertices() - 1, closest_vertex_idx, closure_edge);
		return true;
	}

	return false;
}



VectorXf PoseGraphOptSLAM::get_error_vector(VectorXf Pose_i, VectorXf Pose_j, VectorXf MeasuredTranslatedVector) {

	float x_delta = Pose_j[0] - Pose_i[0];
	float y_delta = Pose_j[0] - Pose_i[0];
	VectorXf error_vector(3);
	
	error_vector[0] = cos(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		- sin(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error_vector[1] = -sin(MeasuredTranslatedVector[2]) * ((cos(Pose_i[2]) * x_delta + sin(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[0]) 
		+ cos(MeasuredTranslatedVector[2]) + ((sin(Pose_i[2]) * x_delta + cos(Pose_i[2]) * y_delta) - MeasuredTranslatedVector[1]);

	error_vector[2] = (Pose_j[2] - Pose_i[2]) - MeasuredTranslatedVector[2];

	return error_vector;
}


// e((x, y, theta)_i, (x, y, theta)_j)
void PoseGraphOptSLAM::build_error_function() {
	
	// Initialize each element in X as an Auto-Diff Object (Equivalent to a variable 'x')
	for (size_t i = 0; i < (PoseDimensions * 2); i++)
		X[i] = AD<float>(0);

	// Declare variables as Independent Variables. And Start Recording (A Gradient Tape Process).
		// Gradient Tape Process: Creates an Operation Sequence
		// Operation Sequence: All operations that depend on the elements of X are recorded on an active tape.
	Independent(X);

	AD<float> DiffX = X[0] - X[1];
	AD<float> DiffY = X[2] - X[3];
	AD<float> DiffTHETA = X[4] - X[5];
	
	float measured_diff_x = 1.f; // Actual value doesnt matter
	float measured_diff_y = 1.f; // Actual value doesnt matter
	float measured_diff_theta = 1.f; // Actual value doesnt matter

	// Set up your error functions that will be Auto-Differentiated
	
	//Differentiate w.r.t.: X[0] = x_j,   X[1] = x_i,   X[2] = y_j,   X[3] = y_i,   X[4] = theta_j,   X[5] = theta_i
	Y[0] = CppAD::cos(measured_diff_theta) * ((CppAD::cos(X[5]) * DiffX + CppAD::sin(X[5]) * DiffY) - measured_diff_x) 
		- CppAD::sin(measured_diff_theta) + ((CppAD::sin(X[5]) * DiffX + CppAD::cos(X[5]) * DiffY) - measured_diff_y);

	Y[1] = -CppAD::sin(measured_diff_theta) * ((CppAD::cos(X[5]) * DiffX + CppAD::sin(X[5]) * DiffY) - measured_diff_x) 
		+ CppAD::cos(measured_diff_theta) + ((CppAD::sin(X[5]) * DiffX + CppAD::cos(X[5]) * DiffY) - measured_diff_y);

	Y[2] = DiffTHETA - measured_diff_theta;

	// Creates f: x -> y and stops tape recording
		// Performs the derivative calculations on the empty x variables.
	ErrorFunction = CppAD::ADFun<float>(X, Y);

}


HbResults PoseGraphOptSLAM::build_linear_system(VectorXf pose_i, VectorXf pose_j, 
	VectorXf MeasuredTranslatedVector, MatrixXf edge_covariance) {
	
	HbResults result;
	VectorXf current_posei_posej(PoseDimensions * 2);
	current_posei_posej << pose_i, pose_j;

	// STEP 1: Set Up Error Function----------------------------------------------
	build_error_function();	
	
	
	// STEP 2: Compute the Jacobian of the Error Function ------------------------
	// Create vector of variables Jacobian will be calculated with respect to ( J(x) ).
	// Holds the value of the corresponding Independent Variable's index.
	// (e.g., 0 = X[0], 1 = X[1], etc.)
	ValueVector WithRespectTo(PoseDimensions * 2);
	for (size_t i = 0; i < (PoseDimensions * 2); i++) {
		//WithRespectTo[i] = (float) i;
		WithRespectTo[i] = current_posei_posej[i];
	}

	// Set up Sparsity Pattern********** Pattern For R (Where J(x) = F(x) * R)
	int numOfRows = (PoseDimensions * 2); // Rows must = num of independent variables X
	int numOfCols = (PoseDimensions * 2); 
	int numOfNonZeroElements = (PoseDimensions * 2); 
	//int numOfNonZeroElements = (PoseDimensions) * (PoseDimensions * 2); // The number of possibly non-zero index pairs in the sparsity pattern
	sparse_rc<SizeVector> SparsityPattern(numOfRows, numOfCols, numOfNonZeroElements);
	for (size_t k = 0; k < numOfNonZeroElements; k++) {
		// Assign a position in the matrix for each non-zero element (indexed 0:numOfNonZeroElements).
		// FIGURE OUT HOW TO CALCULATE THE CORRECT POSITIONS.
		size_t r = k;
		size_t c = k;
		// k: The index (e.g. row[k] = r & col[k] = c) Must be less than numOfNonZeroElements 
		// r: specifies the value assigned to row[k] and Must be less than numOfRows.
		// c: specifies the value assigned to col[k] and Must be less than numOfCols. 
		SparsityPattern.set(k, r, c); 
	}

	// Input the original Sparsity Pattern, apply any alterations then output Sparsity Pattern for J(x)
	bool UseTransposePattern = false;
	bool ConvertToDependencyPattern = false;
	bool UseBoolRepresentation = true;
	sparse_rc<SizeVector> JacobianSparsityPattern; // The output Sparsity Pattern for J(x)
	ErrorFunction.for_jac_sparsity(SparsityPattern, UseTransposePattern, 
		ConvertToDependencyPattern, UseBoolRepresentation, JacobianSparsityPattern);

	// Set up Sparse Matrix***********
	// Specifies which elements of the Jacobian are computed
	sparse_rcv<SizeVector, ValueVector> SparseMatrix(JacobianSparsityPattern);

	// Compute the Sparse Jacobian***********
	CppAD::sparse_jac_work Work; // Stores Information used to reduce computation for future calls. 
	size_t ColorsPerGroup = 2; // The number of colors to undergo the [forward mode auto-diff process] At The Same Time.
	std::string ColoringAlgo = "cppad"; // Algorithm determining which columns will be computed during the same forward sweep.
	size_t n_color = ErrorFunction.sparse_jac_for(ColorsPerGroup, WithRespectTo, 
		SparseMatrix, JacobianSparsityPattern, ColoringAlgo, Work);
	
	// Convert to Eigen format
	Eigen::SparseMatrix<float, Eigen::RowMajor> Jac;
	CppAD::sparse2eigen(SparseMatrix, Jac);

	// Result: Jac is a 3 x 6 Matrix (3 functions, 6 Independent variables)
	// std::cout << "Jac Rows: " << Jac.rows() << std::endl;
	// std::cout << "Jac Cols: " << Jac.cols() << std::endl;
	// std::cout << "Cov Rows: " << edge_covariance.rows() << std::endl;
	// std::cout << "Cov Cols: " << edge_covariance.cols() << std::endl;

	// STEP 3: Create the H Matrix & b vector -----------------

	result.Hii = MatrixXf(Jac).block<3, 3>(0, 0).transpose() * (edge_covariance * MatrixXf(Jac).block<3, 3>(0, 0));
	result.Hjj = MatrixXf(Jac).block<3, 3>(0, 3).transpose() * (edge_covariance * MatrixXf(Jac).block<3, 3>(0, 3));
	result.Hij = MatrixXf(Jac).block<3, 3>(0, 0).transpose() * (edge_covariance * MatrixXf(Jac).block<3, 3>(0, 3));
	result.Hji = MatrixXf(Jac).block<3, 3>(0, 3).transpose() * (edge_covariance * MatrixXf(Jac).block<3, 3>(0, 0));
	result.bi = (MatrixXf(Jac).block<3, 3>(0, 0).transpose() * edge_covariance) * get_error_vector(pose_i, pose_j, MeasuredTranslatedVector);
	result.bj = (MatrixXf(Jac).block<3, 3>(0, 3).transpose() * edge_covariance) * get_error_vector(pose_i, pose_j, MeasuredTranslatedVector);

	return result;
}




bool PoseGraphOptSLAM::front_end(PointCloud current_landmarks) {
	
	Pose pose;
	PoseEdge edge;
	pose.Landmarks = current_landmarks;
	pose.pose = VectorXf::Zero(PoseDimensions);
	pose.TransformationMatrix = MatrixXf::Zero(3, 3);
	pose.TransformationMatrix(2, 2) = 1;

	std::cout << "Point Cloud Overlap Amount: " << calculate_overlap(PreviousLandmarks, current_landmarks) << "\n";
	
	// Set origin node
	if (InitialScan){
		PreviousLandmarks = current_landmarks;
		// Initial rotation should be straight ahead? How to represent that in transformation matrix?
		InitialScan = false;
	}
		
	/* Check for level of overlap between landmark sets of current & previous pose
		Add Pose & Edge to Graph if the amount of overlap is low enough*/
	else if (calculate_overlap(PreviousLandmarks, current_landmarks) > OverlapTolerance) {

		// Dimensions of the scan cloud data
		RotationTranslation rot_trans = icp.runICPSVD(PreviousLandmarks, current_landmarks); 
		MatrixXf R = rot_trans.rotation_matrix;
		VectorXf t = rot_trans.translation_vector;

		// Least Squares version-----------------------------------------------------------
		// VectorXf result = icp.RunICP_LeastSquares(PreviousLandmarks, current_landmarks);
		// std::cout << "Least Squares ICP Result:" << "\n";
		// std::cout << result.transpose() << "\n";
		// //--------------------------------------------------------------------
		
		// std::cout << R.rows() << " & " << t.rows() << "\n"; // Test
		
		PreviousLandmarks = current_landmarks;
		pose.pose[0] = t[0];
		pose.pose[1] = t[1];
		pose.pose[2] = utils::rotationMatrix2DToAngle(R);
		
		// Turn R & t to a Transformation Matrix
		pose.TransformationMatrix.topLeftCorner(2, 2) = R;
		pose.TransformationMatrix.topRightCorner(2, 1) = t;

		//std::cout << pose.TransformationMatrix << "\n"; // Test
		if (std::isnan(pose.pose[0]) || std::isnan(pose.pose[1]) || std::isinf(pose.pose[0]) || std::isinf(pose.pose[1]))
			std::cerr << "ERROR: Bad Pose Data from ICP Front End: " << pose.pose.transpose() << "\n";
	}

	else return false;

	add_pose_to_graph(pose, edge);

	return check_for_loop_closure(pose);
} 



void PoseGraphOptSLAM::optimize() {
	
	// std::cout << "Optimizing" << std::endl;
	update_state_vector();
	VectorXf StateVectorIncrement;
	StateVectorIncrement = VectorXf::Ones(CurrentPoses_n * PoseDimensions);

	HbResults temp_result;
	//Eigen::SparseMatrix<float, Eigen::RowMajor> 
	MatrixXf SparseHessian(CurrentPoses_n * PoseDimensions, CurrentPoses_n * PoseDimensions);
	SparseHessian = MatrixXf::Zero(CurrentPoses_n * PoseDimensions, CurrentPoses_n * PoseDimensions);
	VectorXf coeffVector_b = VectorXf::Zero(CurrentPoses_n * PoseDimensions);
	int iteration = 0;
	

	// While Nodes Not Converged
	while (/* > min_convergence_thresh ||*/ iteration < max_iterations) {

		for (int n = 0; n < Pose_Graph.getNumOfEdges(); n++) { // Calculate & Sum H and b over every edge.

			int edge_index_i = Pose_Graph.getEdgeEnds(n).first;
			int edge_index_j = Pose_Graph.getEdgeEnds(n).second;

			// std::cout << "Edge i Index: " << edge_index_i << " Edge j Index: " << edge_index_j << " Graph Size: " << Pose_Graph.Get_NumOfVertices() << "\n";

			VectorXf pose_i = Pose_Graph.getVertex(edge_index_i).pose;	
			VectorXf pose_j = Pose_Graph.getVertex(edge_index_j).pose;
			PoseEdge edge = Pose_Graph.getEdgeByIndex(n);
			VectorXf cleaned_pose_i(3);
			cleaned_pose_i << pose_i[0], pose_i[1], 1;
			VectorXf translated_vector = edge.TransformationMatrix * cleaned_pose_i;
			//Eigen::SparseMatrix<float, Eigen::RowMajor> edge_covariance = edge.NoiseInfoMatrix;
			
			// Build Linear System
			temp_result = build_linear_system(pose_i, pose_j, translated_vector, edge.NoiseInfoMatrix);

			SparseHessian.block<3, 3>(edge_index_i, edge_index_i) = temp_result.Hii; 
			SparseHessian.block<3, 3>(edge_index_j, edge_index_j) = temp_result.Hjj; 
			SparseHessian.block<3, 3>(edge_index_i, edge_index_j) = temp_result.Hij; 
			SparseHessian.block<3, 3>(edge_index_j, edge_index_i) = temp_result.Hji; 
			coeffVector_b.block<3, 1>(edge_index_i, 0) = temp_result.bi; 
			coeffVector_b.block<3, 1>(edge_index_j, 0) = temp_result.bj; 

			// std::cout << "Hessian: " << "\n" << SparseHessian << "\n";
			// std::cout << "b Vector: " << "\n" << coeffVector_b << "\n";
		}
		

		// Solve the System x = H^-1 * b
		LLT<Eigen::MatrixXf> llt;
		llt.compute(SparseHessian);
		StateVectorIncrement = llt.solve(coeffVector_b); 

		// Update State Vector
		StateVector = StateVector + StateVectorIncrement;

		iteration++;
	}
	convert_state_vector();
}


void PoseGraphOptSLAM::convert_state_vector() {

	int n = 0;
	for (int i = 0; i < StateVector.size(); i = i + PoseDimensions) {
		
		// Re-normalize each angle
		StateVector(i + 2) = utils::normalizeAngleRadians(StateVector(i + 2), true);
		Pose updated_pose = Pose_Graph.getVertex(n);
		updated_pose.TransformationMatrix = vector_to_transformation_matrix(StateVector(i), 
			StateVector(i+1), std::make_pair(StateVector(i+2), rotation_axes[n]));
		Pose_Graph.updateVertexData(n, updated_pose);
		n++;
	}


	// Update all corresponding edge transformation matrices: Loop through poses in graph
	for (int i = 0; i < Pose_Graph.getNumOfVertices(); i++) {

		// Update all adjacent edges for pose i
		for (int j = 0; j < Pose_Graph.getDegree(i); j++) {

			PoseEdge edge = Pose_Graph.getEdge(i, j);
			edge.TransformationMatrix = Pose_Graph.getVertex(i).TransformationMatrix.inverse() 
				* Pose_Graph.getAdjacentVertex(i, j).TransformationMatrix;

			Pose_Graph.updateEdgeData(i, j, edge);
		}
	}
}


//Public --------------------------------------------------------------------------------------------------------
PoseGraphOptSLAM::PoseGraphOptSLAM() {


}

PoseGraphOptSLAM::PoseGraphOptSLAM(int max_nodes, int pose_dimension, int guess_variation) 
	: MaxPoses_n(max_nodes), PoseDimensions(pose_dimension), VariationAroundGuess(guess_variation) {
	
	InitialScan = true;
	CurrentPoses_n = 0;
	NRecentPoses = 5;
	OverlapTolerance = 0.1; // distance in meters
	max_iterations = 100;
	StateVector = VectorXf::Zero(0); // (Assuming a pose dimension of 3 [x, y, theta])
	icp = ICP(ICP_POSE_DIM, ICP_ERR_DIM);
	MapBuilder map_builder();

	std::vector<AD<float>> xs(PoseDimensions * 2);
	std::vector<AD<float>> ys(PoseDimensions); 
	X = xs;
	Y = ys;

	previous_graph_size = 0;
}


void PoseGraphOptSLAM::frontEndInit(int n_recent_poses, float closure_distance) {

	NRecentPoses = n_recent_poses;
	ClosureDistance = closure_distance;
}



Eigen::Tensor<float, 2> PoseGraphOptSLAM::run(PointCloud current_landmarks) {
	
	if (front_end(current_landmarks)) {
		optimize();
		// std::cout << "Uhh... Sending MAP" << std::endl;
		auto start = std::chrono::high_resolution_clock::now();
		// return UpdateMap();
		Eigen::Tensor<float, 2> temp_map = update_map();
		auto end = std::chrono::high_resolution_clock::now();
    	std::cout << "Map Update Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms#########################################################################################" 
			<< std::endl;
		std::cout << "NUM OF POSES: " << Pose_Graph.getNumOfVertices() << std::endl;
		return temp_map;
	}

	// Update map if graph has new node
	if (Pose_Graph.getNumOfVertices() > previous_graph_size) { 
		previous_graph_size = Pose_Graph.getNumOfVertices();
		auto start = std::chrono::high_resolution_clock::now();
		// return UpdateMap();
		Eigen::Tensor<float, 2> temp_map = update_map();
		auto end = std::chrono::high_resolution_clock::now();
    	std::cout << "Map Update Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms#########################################################################################" 
			<< std::endl;
		std::cout << "NUM OF POSES: " << Pose_Graph.getNumOfVertices() << std::endl;
		return temp_map;
	}
		
	return map_structure;
}

void PoseGraphOptSLAM::setMapDimensions(int height, int width) {
	map_height = height;
	map_width = width;
	map_structure_mask = Eigen::Tensor<float, 2>(height, width);
	map_structure = Eigen::Tensor<float, 2>(height, width);
	map_structure.setConstant(0.5);
	map_builder.update2DMapDimensions(height, width);
}

VectorXf PoseGraphOptSLAM::broadcastCurrentPose() {
	 
	if (std::isnan(PreviousPose.pose[0]) || std::isnan(PreviousPose.pose[1]) || std::isinf(PreviousPose.pose[0]) || std::isinf(PreviousPose.pose[1]))
		std::cerr << "ERROR: POSE: " << PreviousPose.pose.transpose() << " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#########################@@@@@@@@@@@@@@@@@@@@" << std::endl;
	return PreviousPose.pose;
}







/*
 * 			TO-DO
 * 			-----
 *  - 
 *   
 *  - 
 * 
 *  */
