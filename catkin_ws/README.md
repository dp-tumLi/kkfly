## RUN the whole project
we recommand you to roslaunch the different node into individual terminal, such that once one of them crashes in RViz, you could just restart it without influence on other nodes. In case if you want to roslaunch all of them by once, you can create a new launch file inside an arbitrary directory, and combine the neccessary launch command into one overall launch file. here is an example you could follow:

   ```
   <launch>
       <include file="$(find your_package_name_folder1)/launch/your_desired_launch_file1.launch"/>
       <include file="$(find your_package_name_folder2)/launch/your_desired_launch_file2.launch"/>
   </launch>
   ```
you could duplicate the `include` syntax for the following launch and specify them in your own launch file.

### RUN them separately:
you will need 6 terminals.
The first step is to compile them with `catkin build` or `catkin_make_isolated`, if you chose the later one, please to forget to source with `source devel_isolated/setup.bash`, otherwise you will need to source by `source devel/setup.bash`. 
    
There is one thing you are recommanded to do, you need to install the octomap plugins for better visulization performance with the command install `apt-get install ros-noetic-octomap-rviz-plugins`.
Open a 1st new terminal under `kkfly/catkin_ws` and run: 


```
source devel/setup.bash
roslaunch simulation simulation.launch
```


Open a 2nd new terminal under `kkfly/catkin_ws` and same directory for later terminals:
```
source devel/setup.bash
roslaunch controller_pkg control.launch
```


on 3rd terminal:
```
source devel/setup.bash
roslaunch filter filter.launch
```


on 4th terminal:
```
source devel/setup.bash
roslaunch path_planner octomap_mapping.launch
```


on 5th terminal:
```
source devel/setup.bash
roslaunch path_planner octomap_mapping3d.launch
```


on 6th terminal:
```
source devel/setup.bash
roslaunch path_planner waypoint_mission.launch
```
