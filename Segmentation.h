/*
*对点云进行分割
*/

#ifndef segmentation_h
#define segmentation_h

#include "Point.h"
#include "PlaneFitting.h"
#include "Plane.h"

#include "kdtree.h"

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <ctime>
#include <iomanip>

namespace td
{
	class Segmentation
	{
	public:
        Segmentation();
        Segmentation(const PointCloud&);
		~Segmentation();

		enum METHOD{RANSAC,BAYSAC,LMEDS,BAYLMEDS};

		//设置点云
        void setInputCloud(const PointCloud&);

		//区域生长分割
        void regionGrow(METHOD , int);

		// multi-RANSAC分割
		void multiRansac();

	    // 单模型拟合
        void singleFitting(METHOD , Plane&);

		// 设置邻域搜索半径
		void setRadius(double r);
	
		// 设置拟合阈值，主要针对RANSAC和BAYSAC拟合
		void setThreshold(double threshold);

		// 点云分割拟合后输出
		void outPut(std::string);

		// 设置k邻域值
		void setKNearest(int);

		// 返回模型个数
		int getModelNum(); 

		// 返回平均误差
		double getAverageError();

        // 返回inliers indices
        const std::vector<PointCloud>& getIndices();

        // 返回面片模型参数
        const std::vector<td::Plane>& getPlaneModels();

	private:
		// k邻域值
		int k;

		// 平均拟合误差
		double averageError;

		PointCloud cloud;//点云
		std::vector<PointCloud> indices;//分割后的点云面片

        std::vector<td::Plane> plane_models; //分割模型参数

		double radius;//邻域搜索半径

		// 拟合阈值,主要针对RANSAC和BaySAC
		double threshold;

		bool selectMethod(PlaneFitting& fit, METHOD method);

	};
}
#endif
