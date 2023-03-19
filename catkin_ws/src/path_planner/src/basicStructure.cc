#include <basicStructure.h>

float SAFER::GetCross( Vertex& p1, Vertex& p2, DirGoals& p)
{
    return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}

bool SAFER::IsPointInMatrix(DirGoals& p, Vertex& p1, Vertex& p2,Vertex& p3,Vertex& p4)
{
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) > 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) > 0;
    //return false;
}