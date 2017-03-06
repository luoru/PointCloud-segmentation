/*
*基本点类型
*罗中飞，2015.12
*/

#ifndef point_h
#define point_h

#include <vector>
#include <memory>

namespace td
{
	class Point
	{
	public:
		Point();
		//构造函数:Point(x,y,z)
		Point(double, double, double);
		~Point();

		//可以用来sort和作为map的key
        friend bool operator<(const Point &_pnta, const Point &_pntb)
        {
            if (_pnta.x<_pntb.x){
                return true;
            }
            else if (_pnta.x>_pntb.x){
                return false;
            }
            else if (_pnta.y<_pntb.y){
                return true;
            }
            else if (_pnta.y>_pntb.y){
                return false;
            }
            else if (_pnta.z<_pntb.z){
                return true;
            }
            else if (_pnta.z>_pntb.z){
                return false;
            }
            else{
                return false;
            }
        }

		//设置点的属性
		void setPoint(double, double, double);
		

	public:
		double x;
		double y;
		double z;
		bool operator==(const Point& pnt);
    };

    typedef  std::vector<td::Point> PointCloud;
    typedef  std::shared_ptr<PointCloud> Ptr;
}
#endif;
