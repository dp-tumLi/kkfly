#include <planner.h>
#include <astar.h>
#include <findGoalatBoundary.h>
#include "../include/frontgoal.h"
#include "../include/wayptrgenerator.h"

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  To Do: Load Trajectory Parameters from file
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to use node handler to get max v and max a params
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_v", max_v_)){
         ROS_WARN("[planner] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/dynamic_params/max_a", max_a_)){
        ROS_WARN("[planner] param max_a not found");
    }
    //
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
}

void BasicPlanner::poseCallback(const nav_msgs::Odometry::ConstPtr& odom){
    current_pos_x_ = odom->pose.pose.position.x;
    current_pos_y_ = odom->pose.pose.position.y;

    // call for test
    // auto var = RealPoseToMapPose();

    // call for test
    // WayPtrGenerator();

    // call for test
    // MapWayPtrToRealWorld();
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
    ROS_ERROR("Failed to get param const_height!"); }

    auto map_processed = goal.DilateObstacle(DilationRadius);
    // test for print the dilated map
    for (auto row : map_processed) {
		for (int cell : row) {
			std::cout << cell << " ";
		}
		std::cout << std::endl;
	}

    // auto next_goal = goal.findGoal(map_processed);
    auto next_goal = goal.findGoal4(map_processed,current_position.first,current_position.second);

 
    std::cout<<"goal pos in map x: "<<next_goal.x<<" y: "<<next_goal.y<<" p: "<<next_goal.priority<<std::endl;

    SAFE::Matrix SafeMatrix = map_processed;
    // SAFE::Matrix SafeMatrix = map_matrix_;

    SAFE::Point goalptr = SAFE::findGoal(SafeMatrix, current_position.first, current_position.second);
    int goalX = goalptr.first;
	int goalY = goalptr.second;

    std::cout<<"goal pos in map x: "<<goalX<<" y: "<<goalY<<std::endl;


	Dijkstra Dijkstra(map_processed);

    // check if the start position really makes sense
    if (Dijkstra.VerifyStart(map_matrix_, current_position.first, current_position.second)){

        std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(current_position.first,current_position.second, goalX, goalY);
        
        if (!path.empty()){
            for (auto wayptr: path){
            waypoints_inMap.push_back(wayptr);

            std::cout<<"waypoint in map x: "<<wayptr.first<<" y :"<<wayptr.second<<std::endl;
            }
        }
    }

    //test for aster


    // Astar::Point goalpoint(goalptr.first, goalptr.second);
    // Astar::Point startpoint(current_position.first, current_position.second);
    // Astar::PathGenerator planner;

    // std::vector<std::vector<int>> AstarMap = map_processed;
    // auto waypoints = planner.planPath(startpoint, goalpoint, AstarMap,5);
    // if(!waypoints.empty()){
    //     for (auto waypoint : waypoints) {
    //     std::cout<<"waypoint in map x: "<<waypoint.x<<" y :"<<waypoint.y<<std::endl;
    //     waypoints_inMap.push_back(std::make_pair(waypoint.x, waypoint.y));
    //     }
    // }
    // else{
    //     std::cout<<"no such a path found!"<<std::endl;
    // }

    // store the wayptr in private attribute for path detection
    waypoints_inMap_ = waypoints_inMap;

    return waypoints_inMap;
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
    for (const auto& eachptr: waypoints_inMap_){
      std::cout<<"map grid value: "<<map_matrix_[eachptr.first][eachptr.second]<<std::endl;
      if (map_matrix_[eachptr.first][eachptr.second] == 100){
        std::cout<<eachptr.first<<","<<eachptr.second<<" has ONE collision!"<<std::endl;

        return false;
      }
      else{
        std::cout<<"we have NOT detected collision!"<<std::endl;
        
        return true;
      }
    }
  };

//////////////////////////////////////////////////////////////////////////////////////////////
// part for test the rotation motion at initial position
bool BasicPlanner::RotationFirst(){
    // Create a MultiDOFJointTrajectory message
    auto rotation = qr_;
    while(ros::ok()){

    
    trajectory_msgs::MultiDOFJointTrajectory traj;

    // Create a MultiDOFJointTrajectoryPoint message
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;

    // Set the time from start to 0
    point.time_from_start = ros::Duration(0.0);

    // Set the position to the current position of the drone
    point.transforms.resize(1);
    point.transforms[0].translation.x = current_pos_x_;
    point.transforms[0].translation.y = current_pos_y_;
    point.transforms[0].translation.z = 6;

    // Set the orientation to the current orientation of the drone

    // rotation.z = rotation.z*ros::Time::now();
    point.transforms[0].rotation = rotation;

    // Set the linear velocity and acceleration to zero
    point.velocities.resize(1);
    point.velocities[0].linear.x = 0.0;
    point.velocities[0].linear.y = 0.0;
    point.velocities[0].linear.z = 0.0;
    point.accelerations.resize(1);
    point.accelerations[0].linear.x = 0.0;
    point.accelerations[0].linear.y = 0.0;
    point.accelerations[0].linear.z = 0.0;

    // Set the angular velocity and acceleration to non-zero values
    point.velocities[0].angular.z = M_PI/2.0; // rotate at pi/2 radians/sec
    point.accelerations[0].angular.z = 0.0;

    // Add the point to the trajectory
    traj.points.push_back(point);

    // Publish the trajectory
    traj.header.frame_id = "world";
    pub_trajectory_rot_.publish(traj);

    }

    return true;

}

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
    // planTrajectoryRotation(firstRotPose, Vel_first, &trajectory);
    // publishTrajectory(trajectory);

    // process a few messages in the background - causes the uavPoseCallback to happen
    // for (int i = 0; i < 20; i++) {
    // ros::spinOnce();  
    // }        

    // // do the general trajectory
    // planTrajectoryForMiddle(waypoints_inWorld, &trajectory);
    // publishTrajectory(trajectory);

    // // maybe also do the last waypoint rotation?
    // return true;
    std::cout<<"test: "<<firstRotPose.x()<<std::endl;
    
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
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;
    // start_vel_4d << 0.0, 0.0, 0.0, 0.0;


    // start.makeStartOrEnd(current_pose_.translation(),
    //                      derivative_to_optimize);

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
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0.15);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, 0.1);
    // solve trajectory
    opt.optimize();
    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////


bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {

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

    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(current_pose_.translation(),
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);

    // add waypoint to list
    vertices.push_back(start);

    std::vector<std::vector<float>> pos_waypt(dimension, std::vector<float>());

    double flyheight = 10;
    if (!nh_.getParam(ros::this_node::getName() + "/waypoints/const_height", flyheight)) {
    ROS_ERROR("Failed to get param const_height!"); }

    auto waypoints_inWorld = MapWayPtrToRealWorld();
	for (auto pathptr: waypoints_inWorld){
		pos_waypt[0].push_back(pathptr.first);
		pos_waypt[1].push_back(pathptr.second);
		pos_waypt[2].push_back(flyheight);
	}
    u_int32_t NumberOfWayPtr = waypoints_inWorld.size();
    std::vector<std::vector<float>> vel_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));
    std::vector<std::vector<float>> acc_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));

    Eigen::VectorXd pos_desired(dimension);
    Eigen::VectorXd vel_desired(dimension);
    Eigen::VectorXd acc_desired(dimension);
    
    int stepsize = 1;
    if (NumberOfWayPtr<=20){
        stepsize = (pos_waypt[0].size()-1)/10;
    }
    else if(NumberOfWayPtr>20 && NumberOfWayPtr<=60){
        stepsize = (pos_waypt[0].size()-1)/20;
    }
    else{
        stepsize = (pos_waypt[0].size()-1)/30;
    }

    for (unsigned int i = 0; i < pos_waypt[0].size()-1; i+=stepsize) 
    {
    std::cout<<"ADD Waypoints"<<std::endl;
    pos_desired << pos_waypt[0][i], pos_waypt[1][i], pos_waypt[2][i];
    vel_desired << vel_waypt[0][i], vel_waypt[1][i], vel_waypt[2][i];
    acc_desired << acc_waypt[0][i], acc_waypt[1][i], acc_waypt[2][i]; 

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos_desired);
    // middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_desired);
    // middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc_desired);

    vertices.push_back(middle);

    middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    // middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    // middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    }

    std::pair<int, int> last_pair = waypoints_inWorld.back();
    goalX_ = last_pair.first;
    goalY_ = last_pair.second;
    Eigen::Vector3d goal_pos, goal_vel;
    goal_vel << 0.0, 0.0, 0.0;
    goal_pos << goalX_, goalY_, 6.0;
    std::cout <<"goal ptr: "<<goal_pos.x()<<""<<goal_pos.y()<<""<<goal_pos.z()<<std::endl;
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

    /******* Configure trajectory *******/
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  To Do: Set up trajectory waypoints
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to
    // - load waypoint definition (pos, vel, acc) per dimension from param file
    // - dynamically set constraints for each (and only where needed)
    // - push waypoints to vertices
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
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

    // double flyheight = 10;
    // if (!nh_.getParam(ros::this_node::getName() + "/waypoints/const_height", flyheight)) {
    // ROS_ERROR("Failed to get param const_height!"); }

    // auto waypoints_inWorld = MapWayPtrToRealWorld();
	// for (auto pathptr: waypoints_inWorld){
	// 	pos_waypt[0].push_back(pathptr.first);
	// 	pos_waypt[1].push_back(pathptr.second);
	// 	pos_waypt[2].push_back(flyheight);
	// }
    // u_int32_t NumberOfWayPtr = waypoints_inWorld.size();
    // std::vector<std::vector<float>> vel_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));
    // std::vector<std::vector<float>> acc_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));

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

    //
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
