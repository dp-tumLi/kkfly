#ifndef FIND_GOAL_AT_BOUNDARY
#define FIND_GOAL_AT_BOUNDARY

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <utility>

using namespace std;

namespace SAFE{

typedef vector<vector<int>> Matrix;
typedef pair<int, int> Point;

bool isValid(const Matrix& map, int x, int y) {
    return x >= 0 && x < map.size() && y >= 0 && y < map[0].size();
}


vector<Point> getNeighbors(const Matrix& map, int x, int y) {
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, 1, 0, -1};
    vector<Point> neighbors;
    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (isValid(map, nx, ny) && map[nx][ny] == 0) {
            neighbors.push_back({nx, ny});
        }
    }
    return neighbors;
}

Point findBoundary(const Matrix& map, int x, int y) {
    queue<Point> q;
    q.push({x, y});
    vector<vector<bool>> visited(map.size(), vector<bool>(map[0].size(), false));
    visited[x][y] = true;
    Point boundary = {-1, -1};
    int maxLength = 0;
    while (!q.empty()) {
        Point curr = q.front();
        q.pop();
        vector<Point> neighbors = getNeighbors(map, curr.first, curr.second);
        for (const auto& neighbor : neighbors) {
            if (!visited[neighbor.first][neighbor.second]) {
                visited[neighbor.first][neighbor.second] = true;
                q.push(neighbor);
            }
        }
        if (neighbors.size() != 2) {
            int length = abs(curr.first - x) + abs(curr.second - y);
            if (length > maxLength) {
                maxLength = length;
                boundary = curr;
            }
        }
    }
    return boundary;
}

Point findGoal(const Matrix& map, int x, int y) {
    Point boundary = findBoundary(map, x, y);
    if (boundary == make_pair(-1, -1)) {
        cerr << "Error: no boundary found" << endl;
        return make_pair(-1, -1);
    }
    queue<Point> q;
    q.push(boundary);
    vector<vector<bool>> visited(map.size(), vector<bool>(map[0].size(), false));
    visited[boundary.first][boundary.second] = true;
    while (!q.empty()) {
        Point curr = q.front();
        q.pop();
        if (map[curr.first][curr.second] == 0) {
            return curr;
        }
        vector<Point> neighbors = getNeighbors(map, curr.first, curr.second);
        for (const auto& neighbor : neighbors) {
            if (!visited[neighbor.first][neighbor.second]) {
                visited[neighbor.first][neighbor.second] = true;
                q.push(neighbor);
            }
        }
    }
    cerr << "Error: no goal found" << endl;
    return make_pair(-1, -1);
}

}

#endif