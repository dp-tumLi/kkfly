#ifndef ASTAR_H
#define ASTAR_H


#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm> 
#include <cstring>
#include <fstream>
#include <functional>
#include <chrono>

using namespace std;

namespace Astar{

    // Define a struct to represent a point in the matrix
    struct Point {
        int x;
        int y;
        Point(int x, int y) : x(x), y(y) {}
    };

    // Define a struct to represent a node in the A* algorithm
    struct Node {
        Point point;
        int f;
        int g;
        int h;
        Node* parent;
        Node(Point p, int f, int g, int h, Node* parent) : point(p), f(f), g(g), h(h), parent(parent) {}
    };

    // Define a function to compute the Euclidean distance between two points
    double distance(Point p1, Point p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }


    class PathGenerator{

        public:
        
        PathGenerator(){};

        // Define a function to get the neighboring points of a point in the matrix
        vector<Point> getNeighbors(Point p, vector<vector<int>>& matrix) {
            vector<Point> neighbors;
            int numRows = matrix.size();
            int numCols = matrix[0].size();
            for (int i = p.x - 1; i <= p.x + 1; i++) {
                for (int j = p.y - 1; j <= p.y + 1; j++) {
                    if (i >= 0 && i < numRows && j >= 0 && j < numCols && !(i == p.x && j == p.y) && matrix[i][j] == 0) {
                        neighbors.push_back(Point(i, j));
                    }
                }
            }
            return neighbors;
        }

        // Define a function to compute the A* algorithm
        vector<Point> aStar(Point start, Point goal, vector<vector<int>>& matrix, int timeout) {
            // Create the open and closed lists for the A* algorithm
            priority_queue<Node*, vector<Node*>, function<bool(Node*, Node*)>> open([](Node* n1, Node* n2) { return n1->f > n2->f; });
            vector<Node*> closed;

            // Create the start node
            Node* startNode = new Node(start, 0, 0, distance(start, goal), nullptr);
            open.push(startNode);

            auto startTime = chrono::steady_clock::now();
            auto endTime = startTime + chrono::milliseconds(timeout);

            // Perform the A* algorithm
            while (!open.empty()&& chrono::steady_clock::now() < endTime) {
                // Get the node with the lowest f value from the open list
                Node* currentNode = open.top();
                open.pop();

                // If the current node is the goal node, we have found a path
                if (currentNode->point.x == goal.x && currentNode->point.y == goal.y) {
                    vector<Point> path;
                    while (currentNode != nullptr) {
                        path.insert(path.begin(), currentNode->point);
                        currentNode = currentNode->parent;
                    }
                    return path;
                }

                // Add the current node to the closed list
                closed.push_back(currentNode);

                // Get the neighboring nodes of the current node
                vector<Point> neighbors = getNeighbors(currentNode->point, matrix);

                // Process each neighboring node
                for (Point neighbor : neighbors) {
                    // If the neighboring node is already in the closed list, skip it
                    bool found = false;
                    for (Node* n : closed) {
                        if (n->point.x == neighbor.x && n->point.y == neighbor.y) {
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        continue;
                    }

                        // Compute the cost of
                    int g = currentNode->g + 1; // The cost of moving to the neighboring node is 1
                    int h = distance(neighbor, goal);
                    int f = g + h;

                    // Create the neighboring node
                    Node* neighborNode = new Node(neighbor, f, g, h, currentNode);

                    // Add the neighboring node to the open list
                    open.push(neighborNode);
                }
            }

            // If we reach this point, there is no path from the start to the goal
            return vector<Point>();
        };

        // Define a function to generate a waypoint in the middle of two obstacles
        Point generateWaypoint(Point p1, Point p2) {
            int x = (p1.x + p2.x) / 2;
            int y = (p1.y + p2.y) / 2;
            return Point(x, y);
        };

        vector<Point> planPath(Point start, Point goal, vector<vector<int>> matrix, int time) {
            // Compute the A* path from the start to the goal
            vector<Point> path = aStar(start, goal, matrix, time);
            // Generate waypoints in the middle of each pair of obstacles in the path
            vector<Point> waypoints;
            if (!path.empty()){
                for (int i = 0; i < path.size() - 1; i++) {
                    Point p1 = path[i];
                    Point p2 = path[i + 1];
                    if (matrix[p1.x][p1.y] == 0 && matrix[p2.x][p2.y] == 0) {
                        Point waypoint = generateWaypoint(p1, p2);
                        waypoints.push_back(waypoint);
                    }
                }
            }
            else{
                std::cout<<"timeout! no path found!"<<std::endl;
            }
            return waypoints;
        };
    };
};

#endif