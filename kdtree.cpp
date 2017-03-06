#include "kdtree.h"
int td::KdTree::kNearestNeighbor(double &_x,double &_y,double &_z)
{
	pnts_.clear();
	nNearestNodes nNN(numberOfNeighbours_);
	nNN.setDistandIndx(radius_);
	nNN.setSearchPnt(_x,_y,_z);
	t_.locateNodes(&nNN,1);
	//cout<<"找到的点数："<<nNN.found<<endl;
	Point p;
	for(int i=1;i<=nNN.found;++i)
	{
		double x, y, z;
		x=nNN.index[i]->pos[0];
		y=nNN.index[i]->pos[1];
		z=nNN.index[i]->pos[2];
		p.setPoint(x, y, z);
		pnts_.push_back(p);
	}
	return pnts_.size();	//返回最临近点个数
}