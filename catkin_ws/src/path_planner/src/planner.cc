#include <planner.h>


BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {

    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_)){
         ROS_WARN("[planner] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_)){
        ROS_WARN("[planner] param max_a not found");
    }

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_rot_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory1", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);


    // subscriber for Odometry
    sub_odom_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::uavOdomCallback, this);

    // subscriber for projected map
    sub_map_ = nh.subscribe("/projected_map",1, &BasicPlanner::mapCallback, this);
    // subscriber for current pos
    sub_current_pose_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::poseCallback, this);

    // publisher of wayptr
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);
}

// liyu part start

void BasicPlanner::mapCallback(const nav_msgs::OccupancyGrid& gridmap){
    map_resolution_ = gridmap.info.resolution;
    map_origin_x_ = gridmap.info.origin.position.x;
    map_origin_y_ = gridmap.info.origin.position.y;


    std::cout<<"map_origin: "<<map_origin_x_<<map_origin_y_<<std::endl;

    // convert map array to matrix
    // I dont understand they should exchanged with others
    auto height = gridmap.info.height;
    auto width = gridmap.info.width;
    std::cout<<"height: "<<height<<" width: "<<width<<" mapsize: "<<gridmap.data.size()<<std::endl;

    std::vector<std::vector<int>> map_temp(height, std::vector<int>(width));

    // TODO:

    // here exists a segmentation fault at we trying to do convert from vector to matrix
    for (int i = 0; i <height; i++){
        for (int j = 0; j < width; j++) {
			map_temp[i][j] = gridmap.data[i * width + j];
            // std::cout<<map_temp[i][j]<<" ";
		}
    }
    std::cout<<"row: "<<map_temp.size()<<" column: "<<map_temp[0].size()<<std::endl;

    map_matrix_ = map_temp;

    // update the vertex of the map in world coordinate
    vertexA_.x = map_origin_x_;
    vertexA_.y = map_origin_y_;
    vertexB_.x = map_origin_x_ + map_matrix_[0].size() * map_resolution_;
    vertexB_.y = map_origin_y_;
    vertexC_.x = map_origin_x_ + map_matrix_[0].size() * map_resolution_;
    vertexC_.y = map_origin_y_ + map_matrix_.size() * map_resolution_;
    vertexD_.x = map_origin_x_;
    vertexD_.y = map_origin_y_ + map_matrix_.size() * map_resolution_;

    std::cout<<"Four Vertex of the map in World coordinate: "<<std::endl;
    std::cout<<"vertex A: "<<vertexA_.x<<" "<<vertexA_.y<<std::endl;
    std::cout<<"vertex B: "<<vertexB_.x<<" "<<vertexB_.y<<std::endl;
    std::cout<<"vertex C: "<<vertexC_.x<<" "<<vertexC_.y<<std::endl;
    std::cout<<"vertex D: "<<vertexD_.x<<" "<<vertexD_.y<<std::endl;


    // update the direction goal in map coordinate
    if (!directionGoals_inWorld_.empty()){
        for (auto &directionGoal: directionGoals_inWorld_){
        directionGoal.x_m = (directionGoal.y - map_origin_y_) / map_resolution_ - 1;
        directionGoal.y_m = (directionGoal.x - map_origin_x_) / map_resolution_ - 1;
        std::cout<<"Dir x_m "<<directionGoal.x_m<<" y_m "<<directionGoal.y_m<<std::endl; 
        }
    }
    
    // check the covery of of goal point
    CheckCovery();
    int tempcount = 1;
    for (auto testobj:directionGoals_inWorld_){
        if (testobj.inside){
            std::cout<<"The "<<tempcount<<" th inside is true"<<std::endl;
        }
        tempcount ++ ;
    }

    
}

void BasicPlanner::InitDirectionGoals(){
    std::vector<double> directionGoals_x, directionGoals_y;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/directionGoals_x", directionGoals_x)){
    ROS_WARN("[planner] param directionGoals_x not found");}
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/directionGoals_y", directionGoals_y)){
    ROS_WARN("[planner] param directionGoals_y not found");}
    size_t number_DirGoals = directionGoals_x.size();
    std::vector<SAFER::DirGoals> temp_goals;
    if (number_DirGoals > 0){
        std::cout<<"START load initial direction goals!"<<std::endl;
        for (size_t i = 0; i < number_DirGoals; i ++){
            temp_goals.push_back(SAFER::DirGoals(directionGoals_x[i],directionGoals_y[i], 1, 1, false, false));
        }
        directionGoals_inWorld_ = temp_goals;
    }    
}

void BasicPlanner::poseCallback(const nav_msgs::Odometry::ConstPtr& odom){
    current_pos_x_ = odom->pose.pose.position.x;
    current_pos_y_ = odom->pose.pose.position.y;

}

std::pair<int, int> BasicPlanner::RealPoseToMapPose(){
    std::pair<int, int> drone_map_pose;

    int drone_map_x = (current_pos_y_ - map_origin_y_) / map_resolution_ - 1;
    int drone_map_y = (current_pos_x_ - map_origin_x_) / map_resolution_ - 1;
    drone_map_pose.first = drone_map_x;
    drone_map_pose.second = drone_map_y;

    // test for current pos:
    std::cout<<"current position in World: x "<<current_pos_x_<<" y "<<current_pos_y_<<std::endl;


    return drone_map_pose;
}

// generate a waypoint using dijkstra in map coordinate
std::vector<std::pair<int, int>> BasicPlanner::WayPtrGenerator(){
    std::vector<std::pair<int, int>> waypoints_inMap;

    // read current position in map coordinate from world coordinate
    std::pair<int, int> current_position = RealPoseToMapPose();
    std::cout<<"current position in map: x "<<current_position.first<<" y "<<current_position.second<<std::endl;
    
    // create goal finder object from FrontGoal class
    Front::FrontGoal goal(map_matrix_, current_position.first, current_position.second);

    int DilationRadius = 1;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/DilationRadius", DilationRadius)) {
    ROS_ERROR("Failed to get param DilationRadius!"); }

    auto map_processed = goal.DilateObstacle(DilationRadius);


    // test for print the dilated map
    // for (auto row : map_processed) {
	// 	for (int cell : row) {
	// 		std::cout << cell << " ";
	// 	}
	// 	std::cout << std::endl;
	// }

    // auto next_goal = goal.findGoal(map_processed);
    // auto next_goal = goal.findGoal4(map_processed,current_position.first,current_position.second);
    // int goalX = next_goal.x;
    // int goalY = next_goal.y;
 
    // std::cout<<"goal pos in map x: "<<next_goal.x<<" y: "<<next_goal.y<<" p: "<<next_goal.priority<<std::endl;

    SAFE::Matrix SafeMatrix = map_processed;
    // SAFE::Matrix SafeMatrix = map_matrix_;

    // SAFE::Point goalptr = SAFE::findGoal(SafeMatrix, current_position.first, current_position.second);



    // PART of choosing the most intelligent goal toward some predefined direction points
    int max_groups = 5;
    int min_length = 6;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/min_length", min_length)){
         ROS_WARN("[planner] param min_length not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_groups", max_groups)){
         ROS_WARN("[planner] param max_groups not found");
    }    
    std::vector<std::pair<int, int>> goalptrs = FindFinalGoal(SafeMatrix, max_groups, min_length
                                                // directionGoals_inWorld
                                                );

    for (auto goalptr: goalptrs){
        int goalX = goalptr.first;
        int goalY = goalptr.second;
        std::cout<<"goal pos in map x: "<<goalX<<" y: "<<goalY<<std::endl;
    }

	Dijkstra Dijkstra(map_processed);

    waypoints_inMap = Dijkstra.DijkstraPlanner(current_position, goalptrs);




    // check if the start position really makes sense
    // if (Dijkstra.VerifyStart(map_matrix_, current_position.first, current_position.second)){

    //     std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(current_position.first,current_position.second, goalX, goalY);
        
    //     if (!path.empty()){
    //         for (auto wayptr: path){
    //         waypoints_inMap.push_back(wayptr);

    //         std::cout<<"waypoint in map x: "<<wayptr.first<<" y :"<<wayptr.second<<std::endl;
    //         }
    //     }
    // }
    // else{
    //     std::cout<<"Verified a failure start position!"<<std::endl;
    // }
    
    

    waypoints_inMap_ = waypoints_inMap;

    return waypoints_inMap;
}


// check now if the predefined goal points are covered by the map
bool BasicPlanner::CheckCovery(){
    bool Checkflag = false;
    // actualize the vertices of the map
    if (!directionGoals_inWorld_.empty()){
        int tempcount = 0;
        for (auto &DirGoal: directionGoals_inWorld_){
            // std::cout<<"inside checkcovery for loop"<<std::endl;
            tempcount ++;
            if (SAFER::IsPointInMatrix(DirGoal, vertexA_, vertexB_, vertexC_, vertexD_)){
                // std::cout<<"inside checkcovery IF loop"<<std::endl;
                DirGoal.inside = true;
                if (map_matrix_[DirGoal.x_m][DirGoal.y_m] == 100 || map_matrix_[DirGoal.x_m][DirGoal.y_m] == 0){
                    // do we need the x_m y_m to be actualized insome callback function such that it is realtime
                    DirGoal.visited = true;
                    std::cout<<"The "<<tempcount<<"inside checkcovery 2nd IF loop"<<std::endl;
                }
                Checkflag = true;
            }
        }
        return Checkflag;
    }

    else{
        return Checkflag;
    }
}


bool BasicPlanner::Termination(){
    bool terminate = false;
    double overall = 170 * 180 / (map_resolution_ * map_resolution_);
    int target = -1;
    size_t count = 0;
    double threshold = 0.7;
    size_t sum = 0;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/map_threshold", threshold)){
    ROS_WARN("[planner] param map_threshold not found");}
    if (map_matrix_.size()> 160){
        for (auto row : map_matrix_){
            count = std::count(row.begin(), row.end(), target);
            sum += count;
            // std::cout <<"how many -1 "<<count<<std::endl;
        }
        
        std::cout<<" quotient "<< sum / overall<<std::endl;
        if (sum/overall < threshold){
            
            terminate = true;
        }
        else{
            terminate = false;
        }

        return terminate;
    }

}


// find the final goal based on some given predefined direction position in World.
std::vector<std::pair<int, int>> BasicPlanner::FindFinalGoal(const vector<vector<int>>& SafeMatrix,
                                                int maxGroups, int minLength
                                                // std::vector<std::pair<int,int>> directionGoals_inWorld
                                                ){
    auto boundaries = SAFER::getBoundaryPoints(SafeMatrix);
    auto segments = SAFER::splitSegments(boundaries, maxGroups, minLength);
    std::vector<std::pair<int, int>> goalcandidate; // goal candidate in map 
    int count = 1;
	for (auto seg: segments){
		auto middlepoint = SAFER::computeSegmentMidpoint(seg);
		std::cout<<"the "<<count<<"th segment middle point x "<<middlepoint.first
        <<" y "<<middlepoint.second<<std::endl;
        goalcandidate.push_back(middlepoint);
        count ++;
    }


    std::pair<int, int> chosenDirGoal;
    int whichGoal = 1;
    for (auto directionGoal: directionGoals_inWorld_){
        std::cout<<"The "<<whichGoal<<" th Goal is chosen"<<std::endl;
        whichGoal ++;
        std::cout<<"The current dir Goal "<<directionGoal.x<<" "<<directionGoal.y<<std::endl;
        if (!directionGoal.visited){
            chosenDirGoal.first = directionGoal.x_m;
            chosenDirGoal.second = directionGoal.y_m;
            std::cout<<"The dir chosen with map coordinate: "<<chosenDirGoal.first<<" "<<chosenDirGoal.second<<std::endl;
            break;
        }
    }

    std::vector<std::pair<int, int>> NearesToFarGoals = SortGoals(goalcandidate, chosenDirGoal);

    return NearesToFarGoals;
}

std::vector<std::pair<int, int>> BasicPlanner::SortGoals(std::vector<std::pair<int, int>> & Goalcandidate, const std::pair<int, int>& Dirgoal){
	std::pair<int, int> referencePoint = Dirgoal; 

	auto distance = [&](const std::pair<int, int>& p) -> double {
		double dx = p.first - referencePoint.first;
		double dy = p.second - referencePoint.second;
		return std::sqrt(dx*dx + dy*dy);
	};
	
	std::sort(Goalcandidate.begin(), Goalcandidate.end(), [&](const std::pair<int, int>& p1, const std::pair<int, int>& p2) -> bool {
    	return distance(p1) < distance(p2);
	});

    //DO we NEED to only remain the first 5 candidates?
    // if (Goalcandidate.size() > 5){
    //     Goalcandidate.resize(5);
    // }

	return Goalcandidate;
}


std::vector<std::pair<int, int>> BasicPlanner::MapWayPtrToRealWorld(){

    auto waypoints_inMap = WayPtrGenerator();
    std::vector<std::pair<int, int>> wayptr_inWorld_Array;

    if (!waypoints_inMap.empty()){
        std::pair<int, int> wayptr_inWorld;

        // path generate
        geometry_msgs::PoseStamped pose;
        nav_msgs::Path path;
        path.header.frame_id = "world";     
        path.header.stamp = ros::Time::now(); 

        for (auto wayptr: waypoints_inMap){
            wayptr_inWorld.first = (wayptr.second + 1)* map_resolution_ + map_origin_x_  ;
            wayptr_inWorld.second = (wayptr.first + 1)* map_resolution_ + map_origin_y_ ;
            
            wayptr_inWorld_Array.push_back(wayptr_inWorld);

            std::cout<<"waypoint in real world x: "<<wayptr_inWorld.first<<" y :"<<wayptr_inWorld.second<<std::endl;

            pose.pose.position.x = wayptr_inWorld.first;
            pose.pose.position.y = wayptr_inWorld.second;
            pose.pose.orientation.w = 1.0; // Set quaternion to default value
            path.poses.push_back(pose);
        }
        path_pub_.publish(path);
    }
    return wayptr_inWorld_Array;
}

bool BasicPlanner::ReachedGoal(int threshold_dis){
    if (abs(goalX_ - current_pos_x_) < threshold_dis && abs(goalY_ - current_pos_y_) < threshold_dis){
        return true;
    }
}

// TODO://///////////////////////////////////////////////////////////////////////////////////
// There exists a task, we should permanently check if the planed way path has collision with 
// new explored map.
bool BasicPlanner::CheckIfPathCollision() {
    bool collision_detected = false;

    for (const auto& eachptr : waypoints_inMap_) {
        // std::cout << "map grid value: " << map_matrix_[eachptr.first][eachptr.second] << std::endl;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = eachptr.first + dx;
                int ny = eachptr.second + dy;
                if (map_matrix_[nx][ny]==100){
                    std::cout << eachptr.first << "," << eachptr.second << " has ONE collision!" << std::endl;
                    collision_detected = true;
                    break;
                }
            }
        }
    }
    

    if (collision_detected) {
        return false;  // collision detected
    } else {
        std::cout << "we have NOT detected collision!" << std::endl;
        return true;  // no collision detected
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
// part for test the rotation motion at initial position

std::pair<Eigen::VectorXd, Eigen::VectorXd> BasicPlanner::RotCommand(){
    // mav_trajectory_generation::Trajectory trajectory;

    double flyheight = 10;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/const_height", flyheight)) {
    ROS_ERROR("Failed to get param const_height!"); }

    auto waypoints_inWorld = MapWayPtrToRealWorld();
    
    std::pair<int, int> firstwaypoint = waypoints_inWorld.front();
    std::pair<int, int> lastwaypoint = waypoints_inWorld.back();
    int dimension_first = 4;

    double yaw_first = 360.0 *M_PI / 180.0;
    int delta_y = lastwaypoint.second - firstwaypoint.second;
    int delta_x = lastwaypoint.first - firstwaypoint.first;
    // yaw_first = std::atan2(delta_y, delta_x);
    Eigen::VectorXd firstRotPose = ComputeWayPtrForTraj(firstwaypoint, flyheight, yaw_first, dimension_first);

    goalX_ = lastwaypoint.first;
    goalY_ = lastwaypoint.second;


    // rotation for the first waypoint
    Eigen::Vector4d Vel_first;
    Vel_first << 0.0,0.0,0.0,0.0;
    
    return std::make_pair(firstRotPose, Vel_first);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> BasicPlanner::RotCommandForNext(){
    // mav_trajectory_generation::Trajectory trajectory;


    Eigen::VectorXd firstRotPose(4);
    firstRotPose << current_pose_.translation(), double (360 * M_PI /180.0);

    // rotation for the first waypoint
    Eigen::Vector4d Vel_first;
    Vel_first << 0.0,0.0,0.0,0.0;
    
    return std::make_pair(firstRotPose, Vel_first);
}

Eigen::VectorXd BasicPlanner::ComputeWayPtrForTraj(const std::pair<int,int>& ptr, double height, double yaw, int dimension){
    Eigen::VectorXd PtrCombination(dimension);
    auto x = ptr.first;
    auto y = ptr.second;
    if (dimension == 3){
        PtrCombination << x, y, height;
        std::cout<<"compute 1"<<std::endl;
    }
    else if (dimension == 4){
        PtrCombination << x, y, height, yaw;
        std::cout<<"compute 2"<<std::endl;

    }

    return PtrCombination;
}

//////////////////////////////////////////////////////////////////////////////////////////////

bool BasicPlanner::planTrajectoryRotation(  const Eigen::VectorXd& goal_pos, 
                                            const Eigen::VectorXd& goal_vel,
                                            mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 4;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;

    start.makeStartOrEnd(start_pos_4d,
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_4d);

    // add waypoint to list
    vertices.push_back(start);

    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);
    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    // add waypoint to list
    vertices.push_back(end);

    double max_v_rot = 0.5;
    double max_a_rot = 0.5;

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_rot, max_a_rot);
    
    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_rot);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_rot);
    // solve trajectory

    opt.optimize();
    opt.getTrajectory(&(*trajectory));
    
}


// TODO: A trajectory generator position and yaw angle constraints along the whole path
bool BasicPlanner::MotionMode(){
    int motion = 1;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/motion", motion)){
    ROS_WARN("[planner] param motion not found");}

    if (motion == 1){
        return true;
    }
    else{
        return false;
    }
}

int BasicPlanner::ReturnDisTolerance(){
    int tol_dis = 8 ;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/tol_dis", tol_dis)){
    ROS_WARN("[planner] param tol_dis not found");}

    return tol_dis;
}


bool BasicPlanner::planTrajectory4D(const std::vector<std::pair<int, int>>& waypoints,
                                    mav_trajectory_generation::Trajectory* trajectory){
    std::vector<std::pair<int, int>> aux_wayptrs = waypoints;
    const int dimension = 4;
    double flyheight = 10;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/const_height", flyheight)) {
    ROS_ERROR("Failed to get param const_height!"); }
    std::vector<std::vector<float>> des_waypoints(dimension, std::vector<float>());
    for (auto pathptr: aux_wayptrs){
		des_waypoints[0].push_back(pathptr.first);
		des_waypoints[1].push_back(pathptr.second);
		des_waypoints[2].push_back(flyheight);
        std::cout<<des_waypoints[0].back()<<" "<<des_waypoints[1].back()<<" "<<des_waypoints[2].back()<<std::endl;
	}
    std::cout<<"finish 1"<<std::endl;
    
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;

    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << des_waypoints[0][1], des_waypoints[1][1], des_waypoints[2][1],
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, current_angular_velocity_[2];

    start.makeStartOrEnd(start_pos_4d,
                         derivative_to_optimize);

    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_4d);

    vertices.push_back(start);



    size_t NumberOfWayPtr = aux_wayptrs.size();
    int stepsize = 1;
    size_t i = 1;
    std::vector<int> remove_num;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/remove_num", remove_num)){
        ROS_WARN("[planner] param remove_num not found");}
        
    if (NumberOfWayPtr<=40 && NumberOfWayPtr>13){
        stepsize = NumberOfWayPtr/10;
        i = remove_num[0];
    }
    else if(NumberOfWayPtr>40 && NumberOfWayPtr<=80){
        stepsize = NumberOfWayPtr/20;
        i = remove_num[1];
    }
    else if(NumberOfWayPtr>80 && NumberOfWayPtr<=140){
        stepsize = NumberOfWayPtr/30;
        i = remove_num[2];
    }
    else if(NumberOfWayPtr>140 && NumberOfWayPtr<=200){
        stepsize = NumberOfWayPtr/40;
        i = remove_num[3];
    }
    else if(NumberOfWayPtr > 200){
        stepsize = 6;
        i = remove_num[4];
    }

    Eigen::Vector4d aux_pose;
    Eigen::VectorXd vel_transl(3);
    for (i; i < des_waypoints[0].size()-1; i += stepsize){
        double yaw = std::atan2(des_waypoints[1][i]-des_waypoints[1][i-1], des_waypoints[0][i]-des_waypoints[0][i-1]);
        
        std::cout<<double(yaw/M_PI*180)<<std::endl;;
        // yaw = 0.0*M_PI/180;
        // yaw = yaw + i*15*M_PI/180;
        aux_pose << des_waypoints[0][i], des_waypoints[1][i], des_waypoints[2][i], yaw;
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, aux_pose);
        
        // if (i < 5){
        //     middle.addConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY, 0.5);
        // }

        vertices.push_back(middle);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);


        std::cout<<"finish 2"<<std::endl;

    }
    
    Eigen::Vector4d goal_pos, goal_vel;
    double yaw_end = std::atan2(des_waypoints[1][des_waypoints[0].size()-1]- des_waypoints[1][des_waypoints[0].size()-2],
                       des_waypoints[0][des_waypoints[0].size()-1] - des_waypoints[0][des_waypoints[0].size()-2]);
    std::cout<<"finish 3"<<std::endl;

    goalX_ = des_waypoints[0][des_waypoints[0].size()-1];
    goalY_ = des_waypoints[1][des_waypoints[1].size()-1];
    goal_pos << goalX_, goalY_, flyheight, yaw_end;

    goal_vel << 0.0, 0.0, 0.0 ,0.0;
    std::cout<<"finish 4"<<std::endl;

    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    vertices.push_back(end);
    std::cout<<"finish 5"<<std::endl;

    double max_v_rot = 2;
    double max_a_rot = 2.5;
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v_rot", max_v_rot)){
         ROS_WARN("[planner] param max_v_rot not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a_rot", max_a_rot)){
        ROS_WARN("[planner] param max_a_rot not found");
    }
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_rot, max_a_rot);
    
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_rot);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_rot);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY, 0.5);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION, 0.5);


    try{
        opt.optimize();
        opt.getTrajectory(&(*trajectory));
        return true;

    } catch(const std::runtime_error& e){
        std::cerr<<"Error during optimization: "<<e.what()<<std::endl;
        return false;

    }
}



//////////////////////////////////////////////////////////////////////////////////////////////


bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(current_pose_.translation(),
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);

    // add waypoint to list
    vertices.push_back(start);


    double flyheight = 10;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/const_height", flyheight)) {
    ROS_ERROR("Failed to get param const_height!"); }
    std::vector<std::vector<float>> pos_waypt(dimension, std::vector<float>());

    auto waypoints_inWorld = MapWayPtrToRealWorld();
	for (auto pathptr: waypoints_inWorld){
		pos_waypt[0].push_back(pathptr.first);
		pos_waypt[1].push_back(pathptr.second);
		pos_waypt[2].push_back(flyheight);
	}
    u_int32_t NumberOfWayPtr = waypoints_inWorld.size();
    std::cout<<"number of waypoints = "<<NumberOfWayPtr<<std::endl;


    Eigen::VectorXd pos_desired(dimension);
    Eigen::VectorXd vel_desired(dimension);
    Eigen::VectorXd acc_desired(dimension);
    
    int stepsize = 1;
    if (NumberOfWayPtr<=40 && NumberOfWayPtr>13){
        stepsize = (pos_waypt[0].size()-1)/10;
    }
    else if(NumberOfWayPtr>40 && NumberOfWayPtr<=80){
        stepsize = (pos_waypt[0].size()-1)/20;
    }
    else if(NumberOfWayPtr>80 && NumberOfWayPtr<=140){
        stepsize = (pos_waypt[0].size()-1)/30;

    }else if(NumberOfWayPtr>140 && NumberOfWayPtr<=200){
        stepsize = (pos_waypt[0].size()-1)/40;
    }

    for (unsigned int i = 0; i < pos_waypt[0].size()-2; i+=stepsize) 
    {
    std::cout<<"ADD Waypoints"<<std::endl;
    pos_desired << pos_waypt[0][i], pos_waypt[1][i], pos_waypt[2][i];

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos_desired);
    vertices.push_back(middle);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);

    }

    std::pair<int, int> last_pair = waypoints_inWorld.back();
    goalX_ = last_pair.first;
    goalY_ = last_pair.second;
    Eigen::Vector3d goal_pos, goal_vel;
    goal_vel << 0.0, 0.0, 0.0;
    goal_pos << goalX_, goalY_, flyheight;
    std::cout <<"goal ptr: "<<goal_pos.x()<<""<<goal_pos.y()<<" "<<goal_pos.z()<<std::endl;
    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);
    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    // add waypoint to list
    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    // solve trajectory
    opt.optimize();
    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    return true;
}

// liyu part end


// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
    
    tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
          
    tf::quaternionMsgToEigen (odom->pose.pose.orientation, q_);
    qr_ = odom->pose.pose.orientation;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(  const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);


    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(current_pose_.translation(),
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);

    // add waypoint to list
    vertices.push_back(start);


    std::vector<std::vector<float>> pos_waypt(dimension, std::vector<float>());
    std::vector<std::vector<float>> vel_waypt(dimension, std::vector<float>());
    std::vector<std::vector<float>> acc_waypt(dimension, std::vector<float>());
    
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/pos_waypt_x", pos_waypt[0])) {
        ROS_ERROR("Failed to get param pos_waypt_x!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/pos_waypt_y", pos_waypt[1])) {
        ROS_ERROR("Failed to get param pos_waypt_y!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/pos_waypt_z", pos_waypt[2])) {
        ROS_ERROR("Failed to get param pos_waypt_z!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/vel_waypt_x", vel_waypt[0])) {
        ROS_ERROR("Failed to get param vel_waypt_x!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/vel_waypt_y", vel_waypt[1])) {
        ROS_ERROR("Failed to get param vel_waypt_y!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/vel_waypt_z", vel_waypt[2])) {
        ROS_ERROR("Failed to get param vel_waypt_z!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/acc_waypt_x", acc_waypt[0])) {
        ROS_ERROR("Failed to get param acc_waypt_x!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/acc_waypt_y", acc_waypt[1])) {
        ROS_ERROR("Failed to get param acc_waypt_y!"); }
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/acc_waypt_z", acc_waypt[2])) {
        ROS_ERROR("Failed to get param acc_waypt_z!"); }

    Eigen::VectorXd pos_desired(dimension);
    Eigen::VectorXd vel_desired(dimension);
    Eigen::VectorXd acc_desired(dimension);

   
        for (unsigned int i = 0; i < pos_waypt[0].size()-1; i++) 
        {
        std::cout<<"ADD Waypoints"<<std::endl;
        pos_desired << pos_waypt[0][i], pos_waypt[1][i], pos_waypt[2][i];
        vel_desired << vel_waypt[0][i], vel_waypt[1][i], vel_waypt[2][i];
        acc_desired << acc_waypt[0][i], acc_waypt[1][i], acc_waypt[2][i]; 

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos_desired);
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_desired);
        middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc_desired);

        vertices.push_back(middle);

        middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
        }


    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero

    // std::pair<int, int> last_pair = waypoints_inWorld.back();

    // goal_pos<<last_pair.first, last_pair.second, 0.0;
    // std::cout<<"goal ptr: "<<goal_pos.x()<<""<<goal_pos.y()<<""<<goal_pos.z()<<std::endl;

    goalX_ = goal_pos.x();
    goalY_ = goal_pos.y();

    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}
