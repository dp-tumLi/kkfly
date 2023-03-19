#include <findGoalatBoundary.h>

bool SAFE::isValid(const Matrix& map, int x, int y) {
    return x >= 0 && x < map.size() && y >= 0 && y < map[0].size();
}

vector<SAFE::Point> SAFE::getNeighbors(const Matrix& map, int x, int y) {
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


SAFE::Point SAFE::findBoundary(const Matrix& map, int x, int y) {
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


SAFE::Point SAFE::findGoal(const Matrix& map, int x, int y) {
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


vector<pair<int, int>> SAFER::getBoundaryPoints(const vector<vector<int>>& matrix) {
    int n = matrix.size(), m = matrix[0].size();
    vector<pair<int, int>> boundaryPoints;
    int dx[] = {1, -1, 0, 0, 1, -1, -1, 1};
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (matrix[i][j] == 0) {
                for (int k = 0; k < 8; k++) {
                    int ni = i + dx[k], nj = j + dy[k];
                    if (ni >= 0 && ni < n && nj >= 0 && nj < m && matrix[ni][nj] == -1) {
                        boundaryPoints.push_back({i, j});
                        break;
                    }
                }
            }
        }
    }
    return boundaryPoints;
}



vector<vector<pair<int, int>>> SAFER::splitSegments(const vector<pair<int, int>>& boundaryPoints, int maxGroups, int minLength) {
    int n = boundaryPoints.size();
    vector<vector<int>> graph(n);
    unordered_map<int, vector<int>> rowPoints;
    unordered_map<int, vector<int>> colPoints;
    for (int i = 0; i < n; i++) {
        rowPoints[boundaryPoints[i].first].push_back(i);
        colPoints[boundaryPoints[i].second].push_back(i);
    }
    for (int i = 0; i < n; i++) {
        int x = boundaryPoints[i].first, y = boundaryPoints[i].second;
        for (int j : rowPoints[x]) {
            if (j != i && abs(boundaryPoints[j].second - y) == 1) {
                graph[i].push_back(j);
            }
        }
        for (int j : colPoints[y]) {
            if (j != i && abs(boundaryPoints[j].first - x) == 1) {
                graph[i].push_back(j);
            }
        }
    }
    vector<vector<pair<int, int>>> segments;
    vector<bool> visited(n, false);
    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            queue<int> q;
            q.push(i);
            visited[i] = true;
            vector<pair<int, int>> segment = {boundaryPoints[i]};
            while (!q.empty()) {
                int curr = q.front();
                q.pop();
                for (int neighbor : graph[curr]) {
                    if (!visited[neighbor]) {
                        visited[neighbor] = true;
                        q.push(neighbor);
                        segment.push_back(boundaryPoints[neighbor]);
                    }
                }
            }
            if (segment.size() >= minLength) {
                segments.push_back(segment);
                if (segments.size() == maxGroups) {
                    break;
                }
            }
        }
    }
    sort(segments.begin(), segments.end(), [](vector<pair<int, int>>& a, vector<pair<int, int>>& b) {
        return a.size() > b.size();
    });
    return segments;
}


std::pair<int, int> SAFER::computeSegmentMidpoint(const std::vector<std::pair<int, int>>& segment) {
    int totalRows = 0, totalCols = 0;
    for (const auto& point : segment) {
        totalRows += point.first;
        totalCols += point.second;
    }
    int numRows = static_cast<int>(segment.size());
    int midRow = std::round(static_cast<double>(totalRows) / numRows);
    int midCol = std::round(static_cast<double>(totalCols) / numRows);
    return {midRow, midCol};
}