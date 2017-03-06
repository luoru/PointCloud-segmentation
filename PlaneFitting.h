/*
*主要用于点云的拟合
*LMS：最小二乘法
*RANSAC：随机抽样一致性
*BaySAC：基于贝叶斯先验概率抽样一致性
*BayLMedS:基于贝叶斯先验概率最小平方中值
*罗中飞，2015.12
*/



#ifndef fitting_h
#define fitting_h


#include "Point.h"
#include "Plane.h"

#include <vector>
#include <set>
#include <ctime>

namespace td
{
	
	class PlaneFitting
	{
	public:
		PlaneFitting();
        PlaneFitting(const td::PointCloud& pc);
		~PlaneFitting();

		//输入点云
        void setInputCloud(const td::PointCloud& pc);


		//RANSAC进行拟合
		bool computeByRANSAC(double );

		//BAYSAC进行拟合
		bool computeByBAYSAC(double );

		//BAYLMEDS进行拟合
		bool computeByBayLMedS();

		// LMEDS
		bool computeByLMedS();

	private:
        PointCloud cloud;
		Plane model;
	public:
		// 返回平面参数
		Plane getModel();
	private:
		// 局内点阈值
		double inliers_threshold;
	public:
		// 返回局内点
		PointCloud getInliers();
		// 返回模型标准阈值
		double getInlierThershold();
		// 返回局外点
		td::PointCloud getOutliers();

	private:
		// 计算点到面的标准差
		double computeStd(Plane& pl, PointCloud& cloud);

		// 最小中值对应的标准差
		// derta = 1.4826[1 + 5/(n-p)]/sqrt(Mj)
		double computeMStd(Plane& pl, PointCloud& cloud, double penalty);
	};
}

#endif;
