/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <planner.h>

#include <iostream>


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    ros::NodeHandle n;
	
    BasicPlanner planner(n);  // instantiate basic planner
    ros::Duration(1.0).sleep();
    ros::Rate loop_rate(2); 
    // define goal point
    Eigen::Vector3d goal_position, goal_velocity;
    goal_position << 7, 0, 7.0;
    goal_velocity << 0.0, 0.0, 0.0;

    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }
    const int tol_dis = 8 ;

    // auto goal_pos_vel = planner.RotCommand();
    mav_trajectory_generation::Trajectory trajectory;
    // planner.planTrajectory(goal_position, goal_velocity, &trajectory);
    // std::cout<<"1111111111111"<<std::endl;
    // planner.publishTrajectory(trajectory);

    // for (int i = 0; i < 100; i++) {
    //     ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    // }
    // trajectory.~Trajectory();
    // Eigen::Vector4d goal_position4, goal_velocity4;
    // goal_position4 << 10, 0, 7, 360.0 *M_PI / 180.0;
    // goal_velocity4 << 0.0, 0.0, 0.0, 0.0;
    // planner.planTrajectoryRotation(goal_position4, goal_velocity4, 
    //                                 &trajectory);
    // std::cout<<"222222222222222"<<std::endl;

    planner.planTrajectory(&trajectory);
    planner.publishTrajectory(trajectory);
    ROS_WARN_STREAM("DONE. First Generation!");

    
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    
    bool executing_rotation_traj = false; // flag variable to keep track of whether the UAV is executing the rotation trajectory or not

    while (ros::ok()) {
        if (planner.ReachedGoal(tol_dis) 
            || !planner.CheckIfPathCollision()
        ){
            if(executing_rotation_traj)
            mav_trajectory_generation::Trajectory trajectory;

            std::cout<<"new traj"<<std::endl;
            std::pair<Eigen::VectorXd, Eigen::VectorXd> goal_pos_vel = planner.RotCommand();

            planner.planTrajectoryRotation(goal_pos_vel.first, goal_pos_vel.second, 
                                                &trajectory);

            std::cout<<"3333333333333333"<<std::endl;
            planner.publishTrajectory(trajectory);
            

            executing_rotation_traj = true; // set the flag to true

            ros::Time start_time = ros::Time::now(); // record the start time of the rotation trajectory

            while (ros::ok() && (ros::Time::now() - start_time).toSec() < 18.0) { // wait for 10 seconds
                ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
                loop_rate.sleep();  // sleep to maintain the desired frequency
            }

            executing_rotation_traj = false; // set the flag to false

            std::cout<<"444444444444444"<<std::endl;
            if (!executing_rotation_traj) { // execute the second trajectory only if the flag is false

                planner.planTrajectory(&trajectory);
                planner.publishTrajectory(trajectory);
                
                std::cout<<"FINISHED 2ND TRAJ"<<std::endl;
            }
            std::cout<<"complete all traj for one path"<<std::endl;
        }

        ros::spinOnce();  // process messages in the background - causes the uavPoseCallback to happen
        loop_rate.sleep();  // sleep to maintain the desired frequency
    }



    return 0;
}

