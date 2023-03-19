/*
 * This header file abstracts the explored map to be a rectangle or polygon,
 * We check the chosen point if it is inside this rectangle. The fundamental 
 * is from https://blog.csdn.net/San_Junipero/article/details/79172260
 */

#ifndef BASIC_STRUCTURE
#define BASIC_STRUCTURE

// a data structure for directional_goal 
namespace SAFER
{
    struct DirGoals{
    int x;
    int y;
    int x_m;
    int y_m;
    bool visited;
    bool inside;
    DirGoals(int x, int y, 
            int x_m, int y_m, 
            bool visited, bool inside){
                this->x = x;
                this->y = y;
                this->x_m = x_m;
                this->y_m = y_m;
                this->visited = visited;
                this->inside  = inside;
            }
    };

    struct Vertex
    {
        float x;
        float y;
        Vertex(float x,float y)
        {
            this->x = x;
            this->y = y;
        }
    };

    //get the cross product of 2 points
    float GetCross( Vertex& p1, Vertex& p2, DirGoals& p);

    //Check the point is inside the rectangle
    bool IsPointInMatrix(DirGoals& p, Vertex& p1, Vertex& p2,Vertex& p3,Vertex& p4);


} // namespace SAFER



#endif