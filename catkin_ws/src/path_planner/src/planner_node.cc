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

    // load direction goals
    planner.InitDirectionGoals(); 
    // load initial param
    // for criteria of verifying if reached goal

    int tol_dis = planner.ReturnDisTolerance();

    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    mav_trajectory_generation::Trajectory trajectory;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> goal_pos_vel_initial = planner.RotCommandForNext();

    planner.planTrajectoryRotation(goal_pos_vel_initial.first, goal_pos_vel_initial.second, 
                                        &trajectory);
    planner.publishTrajectory(trajectory);
    ros::Time start_time_init = ros::Time::now();
    if ((ros::Time::now() - start_time_init).toSec() < 26){
        ros::spinOnce();
        loop_rate.sleep();
    }
    auto aux_ptrs = planner.MapWayPtrToRealWorld();
    planner.planTrajectory4D(aux_ptrs, &trajectory);
    planner.publishTrajectory(trajectory);

    // planner.planTrajectory(&trajectory);
    // planner.publishTrajectory(trajectory);
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

            std::cout<<"new traj"<<std::endl;
            std::pair<Eigen::VectorXd, Eigen::VectorXd> goal_pos_vel = planner.RotCommand();

            planner.planTrajectoryRotation(goal_pos_vel.first, goal_pos_vel.second, 
                                                &trajectory);

            std::cout<<"3333333333333333"<<std::endl;
            planner.publishTrajectory(trajectory);


            executing_rotation_traj = true; // set the flag to true

            ros::Time start_time = ros::Time::now(); // record the start time of the rotation trajectory

            while (ros::ok() && (ros::Time::now() - start_time).toSec() < 20) { // wait for 10 seconds
                ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
                loop_rate.sleep();  // sleep to maintain the desired frequency
                if(!planner.CheckIfPathCollision()){
                    std::cout<<"Current path was check to be collidated!"<<std::endl;
                    break;
                }
            }
            
            executing_rotation_traj = false; // set the flag to false

            std::cout<<"444444444444444"<<std::endl;
            if (!executing_rotation_traj) { // execute the second trajectory only if the flag is false
                
                auto aux_ptrs = planner.MapWayPtrToRealWorld();
                if (planner.MotionMode()){
                    planner.planTrajectory4D(aux_ptrs, &trajectory);
                    std::cout<<"FINISHED 2ND TRAJ"<<std::endl;
                }
                else{
                    if (!planner.planTrajectory4D(aux_ptrs, &trajectory)){
                        // sucessfully used 4D trajectory
                        std::cout<<"used 4d"<<std::endl;

                    }else{
                        planner.planTrajectory(&trajectory);
                        std::cout<<"used 3d"<<std::endl;
                    }
                }

                planner.publishTrajectory(trajectory);
            }
            std::cout<<"complete all traj for one path"<<std::endl;
        }

        ros::spinOnce();  // process messages in the background - causes the uavPoseCallback to happen
        loop_rate.sleep();  // sleep to maintain the desired frequency
    }



    return 0;
}

