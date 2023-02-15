#ifndef FRONT_GOAL_H
#define FRONT_GOAL_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

namespace Front
{

    struct Point
    {
        int x;
        int y;
        int priority;
    };
    
    bool operator<(const Point &a, const Point &b){
            return a.priority > b.priority;
    }

    class FrontGoal{
    public:
    // constructor 
    FrontGoal(std::vector<std::vector<int>> binaryMatrix, int droneX, int droneY)
        :binaryMatrix_(binaryMatrix), droneX_(droneX), droneY_(droneY)
        {};
    
    // do dilation for all obstacles
    std::vector<std::vector<int>> DilateObstacle(const int DilationRadius){
        int rows = binaryMatrix_.size();
        int columns = binaryMatrix_[0].size();

        // Create a copy of the grid map
        std::vector<std::vector<int>> dilated_grid = binaryMatrix_;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                // If the current cell is an obstacle
                if (binaryMatrix_[i][j] == 1) {

                    // This part makes all 8 grids around to be dilated
                    // Mark the neighboring cells as obstacles
                    // for (int x = i - DilationRadius; x <= i + DilationRadius; x++) {
                    //     for (int y = j - DilationRadius; y <= j + DilationRadius; y++) {
                    //         if (x >= 0 && x < rows && y >= 0 && y < columns) {
                    //             dilated_grid[x][y] = 1;
                    //         }
                    //     }
                    // }


                    // This part makes all 4 grids around to be dilated  
                    if (i > 0) {
                        dilated_grid[i - 1][j] = 100;
                    }
                    if (i < rows - 1) {
                        dilated_grid[i + 1][j] = 100;
                    }
                    if (j > 0) {
                        dilated_grid[i][j - 1] = 100;
                    }
                    if (j < columns - 1) {
                        dilated_grid[i][j + 1] = 100;
                    }
                }
            }
        }
        // test for output of dilated map
        // for (auto row : dilated_grid) {
        //     for (int cell : row) {
        //         std::cout << cell << " ";
        //     }
        //     std::cout << std::endl;
        // }
        return dilated_grid;
    }


    // method for finding the frontest goal at boundary betw. 0 and -1
    Point findGoal(){
        binaryMatrix_ = DilateObstacle(DilationRadius_);
        std::priority_queue<Point> frontier;
        std::vector<std::vector<int> >::iterator row;
        std::vector<int>::iterator col;
        int max_x = binaryMatrix_.size()-1;
        int max_y = binaryMatrix_[droneX_].size()-1;
        
        for (row = binaryMatrix_.begin(); row != binaryMatrix_.end(); row ++) {
            for (col = row->begin(); col != row->end(); col ++) {
                if (*col == 0) {

                    int x = row - binaryMatrix_.begin();
                    int y = col - row->begin();
                    for (int dx = (x>0? -1:0); dx <= (x<max_x ? 1:0); ++dx) {
                        for (int dy = (y>0? -1:0); dy <= (y<max_y ? 1:0); ++dy) {
                            if ((dx == 0 || dy  == 0) && (dx+dy!=0)){
                                if (binaryMatrix_[x+dx][y+dy] == -1) {
                                    int priority = std::abs(dx) + std::abs(dy);
                                    frontier.push({x+dx,y+dy, priority});
                                // std::cout<<" x "<<x+dx<<" y "<<y+dy<<" pri "<<priority<<std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!frontier.empty()){
            Point goal = frontier.top();
            frontier.pop();
            return goal;    
        }
        else{
            Point goal = {droneX_,droneY_};
            return goal;
        }
    }    

    private:
        std::vector<std::vector<int>> binaryMatrix_;
        int droneX_, droneY_;
        const int DilationRadius_ = 2;
    };
}
#endif

// FrontGoal(std::vector<std::vector<int>> binaryMatrix_, int M_, int N_, int droneX_, int droneY_);

// Point findGoal();
// Point findGoal(){
//     std::priority_queue<Point> frontier;
//     std::vector<std::vector<int> >::iterator row;
//     std::vector<int>::iterator col;

//     for (row = binaryMatrix_.begin(); row != binaryMatrix_.end(); row ++) {
//         for (col = row->begin(); col != row->end(); col ++) {
//             if (*col == 0) {
//                 int i = row - binaryMatrix_.begin();
//                 int j = col - row->begin();
                
//                 if ((i > 0 && binaryMatrix_[i-1][j] == -1) ||
//                     (i < M_-1 && binaryMatrix_[i+1][j] == -1) ||
//                     (j > 0 && binaryMatrix_[i][j-1] == -1) ||
//                     (j < N_-1 && binaryMatrix_[i][j+1] == -1)) {
//                     int priority = std::abs(i - droneX_) + std::abs(j - droneY_);
//                     frontier.push({i, j, priority});
//                 }
//             }
//         }
//     }
//     Point goal = frontier.top();
//     frontier.pop();

//     return goal;
// }