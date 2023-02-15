#include <planner.h>
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
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::uavOdomCallback, this);

    // subscriber for projected map
    sub_map_ = nh.subscribe("/projected_map",1, &BasicPlanner::mapCallback, this);
    // subscriber for current pos
    sub_current_pose_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::poseCallback, this);
}

// liyu part start

void BasicPlanner::mapCallback(const nav_msgs::OccupancyGrid& gridmap){
    map_resolution_ = gridmap.info.resolution;
    map_origin_x_ = gridmap.info.origin.position.x;
    map_origin_y_ = gridmap.info.origin.position.y;

    std::cout<<"map_origin: "<<map_origin_x_<<map_origin_y_<<std::endl;

    // convert map array to matrix
    auto height = gridmap.info.height;
    auto width = gridmap.info.width;
    std::cout<<"height: "<<height<<" width: "<<width<<" mapsize: "<<gridmap.data.size()<<std::endl;

    std::vector<std::vector<int>> map_temp(height, std::vector<int>(width));

    // TODO:

    // here exists a segmentation fault at we trying to do convert from vector to matrix
    // for (int i = 0; i <height; i++){
    //     for (int j = 0; j < width; j++) {
	// 		map_temp[i][j] = gridmap.data[i * width + j];
	// 	}
    // }

    // maybe because the nav_msgs::OccupancyGrid& gridmap has another way to fit the 
    // processing
    std::copy(gridmap.data.begin(), gridmap.data.end(), map_temp[0].begin());


    map_matrix_ = map_temp;
}

void BasicPlanner::poseCallback(const nav_msgs::Odometry::ConstPtr& odom){
    current_pos_x_ = odom->pose.pose.position.x;
    current_pos_y_ = odom->pose.pose.position.y;

    // call for test
    auto var = RealPoseToMapPose();
    std::cout<<"current position in map: "<<var.first<<var.second<<std::endl;

    // call for test
    WayPtrGenerator();
}

std::pair<int, int> BasicPlanner::RealPoseToMapPose(){
    std::pair<int, int> drone_map_pose;

    int drone_map_x = current_pos_x_ * map_resolution_ + map_origin_x_;
    int drone_map_y = current_pos_y_ * map_resolution_ + map_origin_y_;
    drone_map_pose.first = drone_map_x;
    drone_map_pose.second = drone_map_y;

    return drone_map_pose;
}


void BasicPlanner::WayPtrGenerator(){
    std::pair<int, int> current_position = RealPoseToMapPose();
    Front::FrontGoal goal(map_matrix_, current_position.first, current_position.second);

    auto next_goal = goal.findGoal();
    std::cout<<"x: "<<next_goal.x<<" y: "<<next_goal.y<<" p: "<<next_goal.priority<<std::endl;

    auto goalX = next_goal.x;
	auto goalY = next_goal.y;

	Dijkstra Dijkstra(map_matrix_);

	std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(current_position.first,current_position.second, goalX, goalY);
    
    for (auto wayptr: path){
        goal_x_.push_back(wayptr.first);
        goal_y_.push_back(wayptr.second);
        // std::cout<<wayptr.first<<std::endl;
    }
}



// liyu part end


// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
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


    Eigen::VectorXd pos_desired(dimension);
    Eigen::VectorXd vel_desired(dimension);
    Eigen::VectorXd acc_desired(dimension);

   
        for (unsigned int i = 0; i < pos_waypt[0].size(); i++) 
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
