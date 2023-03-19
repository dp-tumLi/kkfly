/*
 * This header file includes mainly 2 two methods of finding a goal point intelligently,
 * they are actually the different variants of the finding goal methods during the whole
 * project. The namespace also indicates the effort and difference of both methods. We 
 * finally chosen the SAFER one.
 * 
 * 
 * 
 * Main idea of the later one:
 * firstly, find all boundary points grid "0" which as the property that it has at least 
 * one neighbor of grid "0".
 * 
 * secondly, to divide all of them into different groups, which we priorily defined the 
 * minimal number of grids of each group and the maximal number of groups which should be 
 * generated. How to decide who belongs to which group? The boundary points at the map matrix
 * should be able via only horizontally or vertically movenment to have another nearest neighbor
 * of grid "0" (i.e. of course the neighbor is also in the boundary points list), iterate them 
 * until all boundary points have a group or be discard if his group has not satified the 
 * criteria.
 * 
 * thirdly, is the computation of a middle of each segment. this method will be called in the 
 * main plan function, which deliver a list of goal candidate to path planner.
 * 
*/

#ifndef FIND_GOAL_AT_BOUNDARY
#define FIND_GOAL_AT_BOUNDARY

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <utility>
#include <unordered_map>

using namespace std;

namespace SAFE{

    typedef vector<vector<int>> Matrix;
    typedef pair<int, int> Point;

    bool isValid(const Matrix& map, int x, int y);

    vector<Point> getNeighbors(const Matrix& map, int x, int y);

    Point findBoundary(const Matrix& map, int x, int y);

    Point findGoal(const Matrix& map, int x, int y) ;

}


// the safer one boundary goal
namespace SAFER{


    // Boundary segment part:
    vector<pair<int, int>> getBoundaryPoints(const vector<vector<int>>& matrix) ;

    vector<vector<pair<int, int>>> splitSegments(const vector<pair<int, int>>& boundaryPoints, int maxGroups, int minLength);

    std::pair<int, int> computeSegmentMidpoint(const std::vector<std::pair<int, int>>& segment);


}

#endif