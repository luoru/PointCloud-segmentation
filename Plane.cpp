#include "Plane.h"


td::Plane::Plane():
a(0), 
b(0),
c(0),
d(0)
{
	needNum = 3;
}

td::Plane::Plane(double a,double b,double c,double d)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
	needNum = 3;
}

td::Plane::~Plane()
{
}

//通过点集计算参数
void td::Plane::computeFromPoints(PointCloud& cloud)
{
	PointCloud::iterator temIt;

	Eigen::Matrix3d A;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			A(i, j) = 0;
		}
	}

	double meanx, meany, meanz, sumx, sumy, sumz;	//x,y,z的均值
	double detax, detay, detaz;

	sumx = sumy = sumz = 0;
	for (temIt = cloud.begin(); temIt != cloud.end(); ++temIt)
	{
		sumx += temIt->x;
		sumy += temIt->y;
		sumz += temIt->z;
	}
	meanx = sumx / cloud.size();
	meany = sumy / cloud.size();
	meanz = sumz / cloud.size();


	for (temIt = cloud.begin(); temIt != cloud.end(); ++temIt)
	{
		detax = temIt->x - meanx;				//detax[i];
		detay = temIt->y - meany;				//detay[i];
		detaz = temIt->z - meanz;				//detaz[i];
		A(0, 0) += detax*detax;				//deta_xi^2;
		A(0, 1) += detax*detay;				//deta_xi*deta_yi;
		A(0, 2) += detax*detaz;				//deta_xi*deta_zi;

		A(1, 0) += detax*detay;				//deta_xi*deta_yi;
		A(1, 1) += detay*detay;				//deta_yi^2;
		A(1, 2) += detay*detaz;				//deta_yi*deta_zi;

		A(2, 0) += detax*detaz;				//deta_xi*deta_zi;
		A(2, 1) += detay*detaz;				//deta_yi*deta_zi;
		A(2, 2) += detaz*detaz;				//deta_zi^2;
	}

	Eigen::MatrixXd V(1, 3);
	Matrix::minEigenvalueVector(A, V);

	a =  V(0, 0);
	b =  V(0, 1);
	c =  V(0, 2);
	d =  a * meanx + b * meany + c * meanz;
}

//点到平面的距离
double td::Plane::point2plane(Point& pt)
{
	double distance(0);
	distance = fabs(pt.x*a + pt.y*b + pt.z*c - d);
	return distance;
}

//设置平面参数
void td::Plane::setPara(double a, double b, double c, double d)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
}