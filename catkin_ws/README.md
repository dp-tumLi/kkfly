# roslaunch note:
### 08.01-YU:
- 建议使用 catkin_make_isolated 命令进行build，因为template中有package属于非ros类，因此source的时候请使用 source devel_isolated/setup.bash
- make时建议先确定devel和build （非isolated）类文件夹中是否包含内容，建议先清空这两个文件夹
- 如果日后发现非ros类package是可删除的再使用普通的：catkin_make指令
- 建议从之后的更新中使用第另一个branch来满足其他contributer的version需求
- 如有错误请修改此次日志

### 15.01.-Yu:
- 新建了分支yu
- 加入了可用的package: depth_image_pro and octo_mapping，前者生成了cloud points 可以在rostopic中找到/points/raws。
	不过说实话我觉得他根本就没用到，就是单纯的把相机已经有的点remap到了此package上，而我根本没用到这个包所处理的数据....加入的octomap可以在rviz中add一个markerarray添加第二个topic生存voxel map进行可视化，并且octomap生成了topic /output_octomap/ topic是一个binary的octomap的地图信息，可能可以利用此信息进行path planning，加入的teb_local_planner暂时没有成功利用此信息生成path。
- 期望的path planning/navigation package 接收地图信息(不限于octomap-binary file,可以是grid map，不过当前所用的octomap 不支持生成grid map，可能需要重新找目标package），当前的drone current states, goal pos. 这个问题就是对等于他们做的2d map 然后支持A* path 的生成。
- 对于rviz中任何可使用的模块，比如pointcloud2需要找到sensor_msg/PointCloud2 type的 topic才可以生成点，可以在terminal中使用命令 rostopic echo /topic_name 查看是否有信息，使用rostopic typy /topic_name 查看数据类型，或者可以google 一下有特定寻找目标tpye topic的命令。
- 当前的版本运行顺序是 roslaunch simulation simuation.launch, depth_ima_proc point_cloud_xyzrgb.launch, octomap_server octomap_mapping.launch 路径规划模块是没有的当前。然后运行rviz 进行可视化。
- NOTICE:
	所有的published topic都可以通过rostopics lsit 找到，代表他们不受launchfile的控制，只要launchfile里你启动了某个node，那么他node.cpp里所有的topics都会在terminal显示。而在launch file里所作的remap是为了类似于手动subscribe的操作，或者简单说就是赋值。
	
### 17.01.-Yu:
- in case that Lina will help us in the future, the note now will be written in English.
- There are 2 more new packages for Navigation tasks added, due to current progress we havent any docker file which helps to make the ros environment easiely suitable, if you have any problem at catkin_make_isolated, it might be the reason that some of the packages are lost.  
	You can do sudo apt-get install ros-<package name>-noetic if the linux package manager can help you find the package.  
	otherwise, you might need to maunelly download/git clone the package from google repository in you catkin_ws/src, where the custom packages should be added (not your current workspace, the folder might be in you home dic). After cloning, back to catkin_ws and do catkin_make. 
	
- now the problem is:  
	we have already a topic output of the octomap_binary (which contains 3D map informations), we wish to have a navigation part which adapt our binary_octmap topics and could do exploration tasks, which essentialy means, the navigation path towards the larges area where has not been explored yet. 
- My curren progress:  
	I am struggling to make the exploartion/exploration.launch suitable to receive the map topics and also adaptive to our the frame transformation, which is now still not successful. 
- Moreover, the other 2 packages, the teb_local_planner is somehow more difficult to understand and make it adaptive, the global_planner has not be dealed yet.
- The map info is download from https://www.dropbox.com/s/pakfwkoyvnieipe/City_Scenary.zip?dl=0   
which is also be found at the simualtion CMAKELIST file.
####
```bash
git submodule add https://github.com/ethz-asl/mav_trajectory_generation.git
git submodule add https://github.com/ethz-asl/eigen_catkin.git
git submodule add https://github.com/ethz-asl/eigen_checks.git
git submodule add https://github.com/catkin/catkin_simple.git
git submodule add https://github.com/ethz-asl/glog_catkin.git
git submodule add https://github.com/ethz-asl/nlopt.git
git submodule add https://github.com/ethz-asl/yaml_cpp_catkin.git
chmod +x build/nlopt/make_install_nlopt.sh
```
