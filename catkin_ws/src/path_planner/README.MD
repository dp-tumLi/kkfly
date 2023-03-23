## Path Planner instructions
- It is always important to notice, that the path_planner should only be launched after you have launched the node from octomap.
   To simplify the launch work, we recommand you to first read the readme files both at homepage `autsys-projects-kkfly` and workspace `autsys-projects-kkfly/catkin_ws/`. And   `cd your/path/to/catkin_ws`  The ways to launch octomap:
   In one terminal:
   ```
   source devel_isolated/setup.bash
   roslaunch path_planner octomap_mapping.launch
   ```
   In another terminal, sufficient but not neccessary, this is for visualization:
   ```
   source devel_isolated/setup.bash
   roslaunch path_planner octomap_mapping3d.launch
   ```
   
- in a new terminal:  
  `source devel_isolated/setup.bash`  
  `roslaunch path_planner waypoint_mission.launch`  
- any issue please contact [email:](yu.li@tum.de) yu.li@tum.de
## The main functionality of this packages:
- this package provides an exploration orientated trajectory publisher based on [ETH-ASL](https://github.com/ethz-asl/mav_trajectory_generation) with
    the projected map from [Octomap](https://github.com/OctoMap/octomap_mapping), in oder to ultilize this trajectory,
    you should have some controller  package be able to receive this trajectory, in our case, the `controll_pkg` 
    receives the trajetory and publish propeller velocities, more details
    you can find in `kkfly/catkin_ws/src/controller_pkg/`
## Workflow of this package:
- the `planner_node.cc` receives the map information and actualizes serveral important states. And the main logic of the rearrange of 
    different wishes of trajectories.
- the `planner.h/.cc` provides the workflow of the generation of trajectory. based on the following withworks of other ultity packages
- the `basicStructure.h/.cc` splits i.e. predefined the map to be a polygon with 4 vertices and it should be verify some points if its
    inside the explored map.
    - We SETUP some flags i.e. directional goals among the enclose environment, which means our drone do a overall exploration but it should
        go towards some directions until the direction goal/flag are explored. 
    - Indeed you could feel free to modify the private attribute of directional goals which would be contigously visited in the default order
        and go a specific overhead exploration style e.g. "Z" patter, "S" patter or some circle whatever you like.
- the `frontgoal.h/.cc` was an main variant of our goal finder methods. It processes the map to be dilated with a specific grid radius to prevent
    collision during the path planning. There are few of variants of find the frontest goal, such that the it should always be able explore the
    so-called front area. However, since the direction goal is introduced, this goal finder variants are ***discarded***, but they remain here in order
    show the project history for uni-assignment. 
- the `findGoalatBoundary.h/.cc` is another actual goal finder which is used in this package.  
    - it finds all boundary points at boundary where occupied by free space and unknown grids from the projected map.
    - these boundary points are splited into different segments group and return a middle point of each segment as goal candidates.
- the `wayptrgenerator.h/.cc` contains a dijkstra based but enhanced version of path planner, it receives all goal candidate and iteratie them
    until one is accessible. The `findGoalatBoundary.h/.cc` has not checked if the goal is accessible. The path will then modified if it is too 
    close to some surrounding obstacles with a optional detection radius and be shifted.
- the `trajectory_config.yaml`contains some parameter which are tuneable.
- BACK TO OUR `planner.h/.cc`, the work content of this part:
    - general callback and publisher functionalities.
    - update used map informations and robot status.
    - coordinates, states transformation between grid map coordiante and unity world coordinate.
    - sort the goal candiates based on the distance to the current chosen direction goal and send them to the path
        planner, also it checks if the current direction goal is visited.
    - different types of trajectory optimizer incl. 4D and 3D i.e. with/without orientation contraints.
    - publisher of waypoints and trajectory are differently in `rviz` visualizable.
## Parameter of your choice
- we provide also die possibility for some prefered parameter tuning, you can find in `path_planner/config/trajectory_config.yaml`. 
