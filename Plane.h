/*
*平面模型
*平面方程为：ax + by + cz = d, (d > 0);
*a^2 + b^2 + c^2 = 1
*罗中飞，2015.12
*/

#ifndef plane_h
#define plane_h

#include "Model.h"
#include "Point.h"
#include "Matrix.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace td
{
	class Plane:public Model
	{
	public:
		Plane();
		Plane(double, double, double, double);
		~Plane();

		//设置平面参数
		void setPara(double, double, double, double);
		
		//获取平面参数A
		double getA()
		{
			return a;
		}

		//获取平面参数B
		double getB()
		{
			return b;
		}

		//获取平面参数C
		double getC()
		{
			return c;
		}

		//获取平面参数D
		double getD()
		{
			return d;
		}

		//通过点集计算参数，方法为PCA
		void computeFromPoints(PointCloud& );

		//点到平面的距离
		double point2plane(Point&);

	private:
		double a;
		double b;
		double c;
		double d;
	};
}
#endif;
