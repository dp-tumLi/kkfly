#ifndef WAY_PTR_GENERATOR_H
#define WAY_PTR_GENERATOR_H

#include <vector>
#include <queue>
#include <utility>
#include <climits>
#include <string>
#include <algorithm> 

class Dijkstra {
public:
  Dijkstra(const std::vector<std::vector<int>>& map) : map_(map), m_(map.size()), n_(map[0].size()) {}

  std::vector<std::pair<int, int>> findShortestPath(int startX, int startY, int goalX, int goalY) {
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
          if (nx >= 0 && nx < m_ && ny >= 0 && ny < n_ && map_[nx][ny] != 100) {
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
    while (x != startX || y != startY) {
      path.push_back({x, y});
      auto prev = previous[x][y];
      x = prev.first;
      y = prev.second;
    }
    path.push_back({startX, startY});

    // Reverse the path to start from the start position
    std::reverse(path.begin(), path.end());

    return path;
  }

private:
  const std::vector<std::vector<int>>& map_;
  int m_;
  int n_;
};
#endif
