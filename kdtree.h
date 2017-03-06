#ifndef KDTREE_H
#define KDTREE_H


#include <vector>
#include <cmath>
#include "3dKDtree.h"
#include "Point.h"

namespace td
{
	class KdTree
	{
		kdTree t_;
		int numberOfNeighbours_;						//按最近邻域点个数搜索
		double radius_;									//按半径搜索
		PointCloud pnts_;					//存放最近邻域点
	public:
		KdTree(PointCloud &_p) :t_(_p.size())			//建立Kdtree,传如的参数是list数据,继承自Point类，其中有xyz值
		{
			std::vector<td::Point>::iterator it;
			it = _p.begin();

			for (int i = 0; it != _p.end(); ++it, ++i)
			{
				t_.store(it->x, it->y, it->z, i);
			}
			t_.treeBalance();							//调整树
		}

		void setNumberOfNeighbours(int _num)			//按最近邻域点搜索
		{
			numberOfNeighbours_ = _num;
			radius_ = 5.0;								//默认1.0米,具有500+个点，搜索半径太大没意义
		}

		void setNeighboursRadius(double _radius)		//按半径搜索
		{
			radius_ = _radius;
			numberOfNeighbours_ = 1000;					//1平米点数约有700+点
		}
		int kNearestNeighbor(double &_x, double &_y, double &_z);	//搜索最近n个邻域点,其本身也被包括在内&返回最临近点个数

		std::vector<td::Point>& getNearestNeighbor()				//取最临近值
		{
			return pnts_;
		}
	};
}
#endif