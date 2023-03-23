#include <wayptrgenerator.h>

Dijkstra::Dijkstra(const std::vector<std::vector<int>>& map) : map_(map), m_(map.size()), n_(map[0].size()) {}



std::vector<std::pair<int, int>> Dijkstra::DijkstraPlanner(const std::pair<int, int>& start, 
                                                        const std::vector<std::pair<int, int>>& goalcandidate){
    
    std::vector<std::pair<int, int>> waypoints;

    for(auto goal: goalcandidate){
        std::vector<std::pair<int, int>> temp_path = findShortestPath(start.first, start.second, goal.first, goal.second);
        
        if (!temp_path.empty()){
            // waypoints = modifyPath1(map_, temp_path);

            // test for new part of modification
            waypoints = temp_path;
            auto gridmap = map_;
            modifyWaypoints(gridmap,waypoints,2);

            break;
        }
    }
    for (auto ptr: waypoints){
      std::cout<<"Dijkstra modified: "<<ptr.first<<" "<<ptr.second<<std::endl;
    }
    return waypoints;
}


// of course the Dijkstra is not the shortest approach.
std::vector<std::pair<int, int>> Dijkstra::findShortestPath(int startX, int startY, int goalX, int goalY) {
    // Initialize the distances and the previous node for each cell
    std::vector<std::vector<int>> distances(m_, std::vector<int>(n_, INT_MAX));
    std::vector<std::vector<std::pair<int, int>>> previous(m_, std::vector<std::pair<int, int>>(n_, {-1, -1}));
    distances[startX][startY] = 0;

    // Use a priority queue to store the cells to be processed
    std::priority_queue<std::pair<int, std::pair<int, int>>> pq;
    pq.push({0, {startX, startY}});

    // Dijkstra algorithm
    while (!pq.empty()) {
      auto current = pq.top();
      pq.pop();
      int x = current.second.first;
      int y = current.second.second;

      if (x == goalX && y == goalY) {
        break;
      }

      // Check the neighbors
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          int nx = x + dx;
          int ny = y + dy;

          // Check if the neighbor is within the map and not an obstacle
          if (nx >= 0 && nx < m_ && ny >= 0 && ny < n_ && map_[nx][ny] != 100
        //    && map_[nx][ny] != -1
           ) {
            int newDistance = distances[x][y] + 1;
            if (newDistance < distances[nx][ny]) {
              distances[nx][ny] = newDistance;
              previous[nx][ny] = {x, y};
              pq.push({-newDistance, {nx, ny}});
            }
          }
        }
      }
    }

    // Build the shortest path
    std::vector<std::pair<int, int>> path;
    int x = goalX;
    int y = goalY;
    
    // test for what if goal not found
    std::cout<<"Planned last point before Goal X: "<<previous[x][y].first
    <<" Planned last point before Goal Y: "<<previous[x][y].second<<std::endl;

    auto Second_last = previous[x][y];
    if (abs(Second_last.first-goalX)<2 && abs(Second_last.second-goalY)<2)
    {
      while (x != startX || y != startY) {
      path.push_back({x, y});
      auto prev = previous[x][y];
      x = prev.first;
      y = prev.second;
      }
      path.push_back({startX, startY});
      // Reverse the path to start from the start position
      std::reverse(path.begin(), path.end());

      for (auto ptr: path){
        std::cout<<"Dijkstra original: "<<ptr.first<<" "<<ptr.second<<std::endl;
      }
    }
    else{
      std::cout<<"FAILED plan a path in map!"<<std::endl;
    }
  
    return path;
  } 



bool Dijkstra::VerifyPath(std::vector<std::pair<int, int>> found_path){
    if (found_path.size()!=0){
        std::cout<<"find a path!!"<<std::endl;

        return false;
    }
    else{
        std::cout<<"not such a path found!!"<<std::endl;

        return true;
    }
}

bool Dijkstra::VerifyStart(const std::vector<std::vector<int>>& map, int StartX,int StartY){
    if (map[StartX][StartY] != -1 && map[StartX][StartY] != 0 ){
        std::cout<<"Your initial position is invalid!!"<<std::endl;

        return false;
    }
    else{
        std::cout<<"You current at a safe place!!"<<std::endl;

        return true;
    }

}


/// new part of modification of path
const int FREE_SPACE = 0;
const int OBSTACLE = 100;
const int UNKNOWN = -1;


// function to check if a given point is safe
bool Dijkstra::isSafe(std::vector<std::vector<int>>& map, int x, int y, int radius) {
    int n = map.size();
    int m = map[0].size();
    for (int i = std::max(0, x - radius); i <= std::min(n - 1, x + radius); i++) {
        for (int j = std::max(0, y - radius); j <= std::min(m - 1, y + radius); j++) {
            if (map[i][j] == OBSTACLE) {
                int dx = i - x;
                int dy = j - y;
                if (dx * dx + dy * dy <= radius * radius) {
                    return false;
                }
            }
        }
    }
    return true;
}

// function to shift a point to the nearest safe location
std::pair<int, int> Dijkstra::shiftPoint(std::vector<std::vector<int>>& map, int x, int y, int radius) {
    int n = map.size();
    int m = map[0].size();
    for (int r = 1; r <= radius; r++) {
        for (int i = std::max(0, x - r); i <= std::min(n - 1, x + r); i++) {
            for (int j = std::max(0, y - r); j <= std::min(m - 1, y + r); j++) {
                if (map[i][j] == FREE_SPACE && isSafe(map, i, j, radius)) {
                    return std::make_pair(i, j);
                }
            }
        }
    }
    // if no safe location is found, return the original point
    return std::make_pair(x, y);
}



// function to modify a vector of waypoints to make them safe
void Dijkstra::modifyWaypoints(std::vector<std::vector<int>>& map, std::vector<std::pair<int, int>>& waypoints, int radius) {
    // add the start and goal points to the vector if they are not already there
    // if (waypoints.empty() || waypoints.front() == make_pair(0, 0)) {
    //     waypoints.insert(waypoints.begin(), make_pair(0, 0));
    // }
    // if (waypoints.back() != make_pair(map.size() - 1, map[0].size() - 1)) {
    //     waypoints.push_back(make_pair(map.size() - 1, map[0].size() - 1));
    // }
    // iterate over the waypoints and shift them to safe locations
    int i = 1;
    while (i < waypoints.size() - 1) {
        std::pair<int, int> prev = waypoints[i - 1];
        std::pair<int, int> curr = waypoints[i];
        std::pair<int, int> next = waypoints[i + 1];
        // check if the current point is safe
        if (isSafe(map, curr.first, curr.second, radius)) {
            i++;
        } else {
            // shift the current point to a safe location
            std::pair<int, int> shifted = shiftPoint(map, curr.first, curr.second, radius);
            // if the shifted point is the same as the previous point or the next point, remove the current point
            if (shifted == prev || shifted == next) {
                waypoints.erase(waypoints.begin() + i);
            } else {
                // otherwise, replace the current point with the shifted point
                waypoints[i] = shifted;
                i++;
              }
            }
        }
}