/*
 * This is the Path Planner method. 
 * We reinforced the DijkstraPlanner to be called serveral times until 
 * it finally find a path which is not empty, and not collidates with the obstacle.
 * 
 * We do this because of the complexity of environment and also as a function to 
 * recover the unsense goal points. In the Goal candidates from the FindGoalatBoundary.h,
 * they are not verified if they are actually accessible, because some area inside a
 * room will also be projected to be -1, or some areas are definitely surrouded by 
 * obstacles but with some unknown -1 in the middle; 
 * 
 * Therefore, the Dijkstra algorithm itself will return empty if it cant find one path. 
 * 
 * then we will iterate all goal candidates
 * with priority which are declaried in the planner.cc. 
 * 
 * Moreover, we do modification with 
 * some reliable detection radius of the path, it detects if the current waypoint has
 * some obstacle inside the radius, otherwise, it will be shifted to a safe grid.
 * 

*/

#ifndef WAYPTR_GENERATOR_H
#define WAYPTR_GENERATOR_H

#include <vector>
#include <queue>
#include <utility>
#include <climits>
#include <string>
#include <algorithm>
#include <iostream> 
#include <math.h>

class Dijkstra {
public:
  Dijkstra(const std::vector<std::vector<int>>& map) ;

  std::vector<std::pair<int, int>> DijkstraPlanner(const std::pair<int, int>& start, 
                                                            const std::vector<std::pair<int, int>>& goalcandidate);

  std::vector<std::pair<int, int>> findShortestPath(int startX, int startY, int goalX, int goalY);

  bool VerifyPath(std::vector<std::pair<int, int>> found_path);
  
  bool VerifyStart(const std::vector<std::vector<int>>& map, int StartX,int StartY);

  std::vector<std::pair<int, int>> modifyPath1(const std::vector<std::vector<int>>& grid_map, 
                                              const std::vector<std::pair<int, int>>& path);

  // new modify part:
  void modifyWaypoints(std::vector<std::vector<int>>& map, std::vector<std::pair<int, int>>& waypoints, int radius);
  std::pair<int, int> shiftPoint(std::vector<std::vector<int>>& map, int x, int y, int radius);
  bool isSafe(std::vector<std::vector<int>>& map, int x, int y, int radius);



private:
  const std::vector<std::vector<int>>& map_;
  int m_;
  int n_;


};
#endif