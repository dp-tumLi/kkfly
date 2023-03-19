/*
 * This is a first Variant of find a goal at front area
 * REMINDER: according to current usage, only the dilation is used. Other
 * Goal finder methods are substituted by findGoalatBoundary.h
 * 
 * In order to show the progress of our project, the explanation of functionality 
 * of discarded methods will also be declared.
 * 
 * findGoal:
 * is our first generation of find a intuitive goal, it uses a naive search to seek
 * a boundary point as a front goal, it should nominally give a priority if the 
 * current candidate has the clost distance to current states, but it has not been 
 * fine achieved, therefore it was only used during the debug phase.
 * 
 * findGoal4:
 * This is a method of finding the longest boundary point as goal, it was proposed by 
 * Chatgpt, and adapted into C++ and Class style. It tests very well at the early time
 * of our testing, and it always find the longest boundary mididle points, which inspires
 * us a lot, such that we based on it adapt our final approach of find list of boundary
 * points as candidates.

*/

#ifndef FRONT_GOAL_H
#define FRONT_GOAL_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

namespace Front
{

    struct Point
    {
        int x;
        int y;
        int priority;
    };
    
    bool operator<(const Point &a, const Point &b);

    class FrontGoal{
    public:
    // constructor 
    FrontGoal(std::vector<std::vector<int>> binaryMatrix, int droneX, int droneY);
    
    // do dilation for all obstacles
    std::vector<std::vector<int>> DilateObstacle(const int DilationRadius);


    // method for finding the frontest goal at boundary betw. 0 and -1
    Point findGoal(std::vector<std::vector<int>>& binaryMatrix);

    bool isBoundary(const std::vector<std::vector<int>>& matrix, int i, int j);    

    Point findGoal4(const std::vector<std::vector<int>>& matrix, int x, int y);


    private:
        std::vector<std::vector<int>> binaryMatrix_;
        int droneX_, droneY_;
        const int DilationRadius_ = 1;
    };
}
#endif

