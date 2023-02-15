#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>


class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(double max_v);

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

    // liyu part added

    // callback function to receive a map
    void mapCallback(const nav_msgs::OccupancyGrid& gridmap);
    // callback receive pose
    void poseCallback(const nav_msgs::Odometry::ConstPtr& odom);
    // convert real position in world frame to map coordinate
    std::pair<int, int> RealPoseToMapPose();
    // generate Wayptr
    void WayPtrGenerator();

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_map_; // liyu
    ros::Subscriber sub_current_pose_;

    ros::NodeHandle& nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;

    double current_pos_x_;
    double current_pos_y_;

    std::vector<double> goal_x_;
    std::vector<double> goal_y_;
    std::vector<std::vector<int>> map_matrix_;
};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H
