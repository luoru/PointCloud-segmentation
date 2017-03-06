#include "Point.h"
#include <cmath>

td::Point::Point():
    x(0),
    y(0),
    z(0)
{

}

td::Point::Point(double x,double y,double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

td::Point::~Point()
{
}

//设置点的属性
void td::Point::setPoint(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}


bool td::Point::operator==(const Point& pnt)
{
    if (x == pnt.x)
        if (y == pnt.y)
            if (z == pnt.z)
                return true;
    return false;
}
