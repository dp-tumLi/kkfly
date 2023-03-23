/*
 * planner.h provides different diverse methods of trajectory generation,
 * callback functions for actualization of map, drone state, map state 
 * checking, coordinate transformation and neccessary assistant function
 * of some different purpose.
 * 
 * we provide a motion combination.
 * A) The basic motion, composed with do first 360° rotation for mapping 
 * neighbouring buildings, and a 3D trajectory for generic movenment from
 * current position to some goal position
 * 
 * B) a high level planned motion, but the the first 360° rotation will be
 * not neglected, then with the 4D trajectory incl yaw movement, such that
 * quadrotor always towards the direction where it flys like humand does. 
 * Which leads to a besser mapping capacity.
 *
 */

#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <findGoalatBoundary.h>
#include <frontgoal.h>
#include <wayptrgenerator.h>
#include <basicStructure.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(double max_v);

    bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    bool planTrajectoryRotation(const Eigen::VectorXd& goal_pos, 
                                const Eigen::VectorXd& goal_vel,
                                mav_trajectory_generation::Trajectory* trajectory);

    bool planTrajectoryForMiddle(std::vector<std::pair<int, int>> middlewaypoints,
                                mav_trajectory_generation::Trajectory* trajectory);
    // liyu part added

    // callback function to receive a map
    void mapCallback(const nav_msgs::OccupancyGrid& gridmap);
    // callback receive pose
    void poseCallback(const nav_msgs::Odometry::ConstPtr& odom);
    // convert real position in world frame to map coordinate
    std::pair<int, int> RealPoseToMapPose();

    // generate Wayptr in map coordinate
    std::vector<std::pair<int, int>> WayPtrGenerator();

    // convert waypoints into real world coordinate
    std::vector<std::pair<int, int>>  MapWayPtrToRealWorld();

    // check if goal reached
    bool ReachedGoal(int threshold_dis);
    bool CheckIfPathCollision();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> RotCommand();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> RotCommandForNext();
    // other goal function 
    std::vector<std::pair<int, int>> FindFinalGoal(const std::vector<std::vector<int>>& SafeMatrix,
                                                int maxGroups, int minLength 
                                                // std::vector<std::pair<int,int>> directionGoals_inWorld
                                                );
    
    std::vector<std::pair<int, int>> SortGoals(std::vector<std::pair<int, int>> & Goalcandidate, const std::pair<int, int>& Dirgoal);

    // some tool functions
    Eigen::VectorXd ComputeWayPtrForTraj(const std::pair<int,int>& ptr, double height, double yaw, int dimension);
    
    void InitDirectionGoals();
    bool CheckCovery();
    
    bool planTrajectory4D(const std::vector<std::pair<int, int>>& waypoints,
                                    mav_trajectory_generation::Trajectory* trajectory);

    bool planYaw(const std::vector<std::pair<int, int>>& waypoints,
                                    mav_trajectory_generation::Trajectory* trajectory);
                                    
    bool MotionMode();
    int ReturnDisTolerance();
    bool Termination();

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Publisher pub_trajectory_rot_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_map_; // liyu
    ros::Subscriber sub_current_pose_;
    ros::Publisher path_pub_;

    ros::NodeHandle& nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    Eigen::Quaterniond q_;
    geometry_msgs::Quaternion qr_; 
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;

    double current_pos_x_;
    double current_pos_y_;

    std::vector<std::vector<int>> map_matrix_;
    double goalX_ = 35;
    double goalY_ = -9;
    std::vector<std::pair<int, int>> waypoints_inMap_;
    // std::vector<std::pair<int, int>> waypoints_inWorld_;
    std::vector<SAFER::DirGoals> directionGoals_inWorld_{SAFER::DirGoals(100, 0, 1, 1, false, false),
                                                        SAFER::DirGoals(100, -100, 1, 1, false, false),
                                                        SAFER::DirGoals(0, -100, 1, 1, false, false),
                                                        SAFER::DirGoals(-100, -100, 1, 1, false, false)
                                                        };
    SAFER::Vertex vertexA_ = SAFER::Vertex(float(map_origin_x_), float(map_origin_y_));
    SAFER::Vertex vertexB_ = SAFER::Vertex(float(map_origin_x_), float(map_origin_y_));
    SAFER::Vertex vertexC_ = SAFER::Vertex(float(map_origin_x_), float(map_origin_y_));
    SAFER::Vertex vertexD_ = SAFER::Vertex(float(map_origin_x_), float(map_origin_y_));
};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H
