#include "Segmentation.h"



td::Segmentation::Segmentation()
: radius(0)
, threshold(0)
, k(0)
, averageError(0)
{
}

td::Segmentation::Segmentation(const PointCloud& in)
{
    cloud = in;
}

td::Segmentation::~Segmentation()
{
}

void td::Segmentation::setInputCloud(const PointCloud& in)
{
    cloud = in;
}

//区域生长分割,RANSAC拟合
void td::Segmentation::regionGrow(METHOD method, int k)
{																	//建立树
	
	
	//Step1:准备k邻域数据
	KdTree tree(cloud);
	//建立空间坐标索引
	/*std::map<Point, int> index_map;*/
	std::map<Point, bool> label;
	
	//标签初始化
	for (size_t index = 0; index < cloud.size(); ++index)
	{
		label[cloud[index]] = false;
		/*index_map[cloud[index]] = index;*/
	}

	//随机种子点
	for (size_t index = 0; index < cloud.size();index++)
	{
		if (label[cloud[index]])
			continue;
		PointCloud neighbors;            //邻域点集
		tree.setNumberOfNeighbours(k);
		//tree.setNeighboursRadius(radius);//邻域半径
		tree.kNearestNeighbor(cloud[index].x, cloud[index].y, cloud[index].z);
		neighbors = tree.getNearestNeighbor();

		//开始进行拟合,找到种子面
		PlaneFitting fit;
		fit.setInputCloud(neighbors);
		bool state(false);
		state = selectMethod(fit, method);
		if (!state)
			continue;
		//获取平面参数，及局内点
		Plane pl;
		pl = fit.getModel();
		PointCloud inliers;
		inliers = fit.getInliers();

		//对邻域点进行标记
		for (size_t in_i = 0; in_i < inliers.size(); ++in_i)
		{
			label[inliers[in_i]] = true;
		}

		//对局内点进行区域生长
		int n = 0;
		//最好不要使用迭代器进行遍历
		for (size_t i = 0; i < inliers.size(); ++i)
		{
			tree.kNearestNeighbor(inliers[i].x, inliers[i].y, inliers[i].z);
			PointCloud candidate = tree.getNearestNeighbor();
			for (PointCloud::iterator can_it = candidate.begin(); can_it != candidate.end(); ++can_it)
			{
				double dis = pl.point2plane(*can_it);
				if (dis > fit.getInlierThershold() || label[*can_it] == true)
				{
					continue;
				}
				++n;
				inliers.push_back(*can_it);								//加入局内点
				label[*can_it] = true;
                if (n % k == 0)
				{
					fit.setInputCloud(inliers);
					state = selectMethod(fit, method);
					pl = fit.getModel();								//更新平面参数
				}
			}
		}
		indices.push_back(inliers);
        plane_models.push_back(pl);
		double currentError = 0;
		for (size_t i = 0; i < inliers.size(); i++)
		{
			currentError += pow(pl.point2plane(inliers[i]), 2);
		}
		currentError /= inliers.size();
		currentError = sqrt(currentError);
		averageError += currentError;
	}
	averageError /= indices.size();
}

int td::Segmentation::getModelNum()
{
	return indices.size();
}


// 设置邻域搜索半径
void td::Segmentation::setRadius(double r)
{
	this->radius = r;
}


// 设置拟合阈值，主要针对RANSAC和BAYSAC拟合
void td::Segmentation::setThreshold(double threshold)
{
	this->threshold = threshold;
}


bool td::Segmentation::selectMethod(PlaneFitting& fit, METHOD method)
{
	bool state;
	switch (method)
	{
	case RANSAC:
		state = fit.computeByRANSAC(threshold);
		break;
	case BAYSAC:
		state = fit.computeByBAYSAC(threshold);
		break;
	case LMEDS:
		state = fit.computeByLMedS();
		break;
	case BAYLMEDS:
		state = fit.computeByBayLMedS();
		break;
	default:
		break;
	}
	return state;
}


// 点云分割拟合后输出
void td::Segmentation::outPut(std::string path)
{
	std::ofstream outfile;
	outfile.open(path, std::ios::out);
	
	srand((unsigned)time(NULL));
	for (size_t i = 0; i < indices.size(); i++)
	{
		//附上随机颜色
		unsigned int r = (int)((double)rand() / ((RAND_MAX + 1.0) / (255 - 100 + 1.0)) + 100);
		unsigned int g = (int)((double)rand() / ((RAND_MAX + 1.0) / (255 - 100 + 1.0)) + 100);
		unsigned int b = (int)((double)rand() / ((RAND_MAX + 1.0) / (255 - 100 + 1.0)) + 100);
		for (size_t j = 0; j < indices.at(i).size(); j++)
		{
			outfile << std::setw(15);
			outfile << std::right;
			outfile << std::fixed;
			outfile << indices.at(i).at(j).x;
			outfile << std::setw(15);
			outfile << std::right;
			outfile << indices.at(i).at(j).y;
			outfile << std::setw(15);
			outfile << std::right;
			outfile << indices.at(i).at(j).z;
			outfile << std::setw(6);
			outfile << std::right;
			outfile << r;
			outfile << std::setw(6);
			outfile << std::right;
			outfile << g;
			outfile << std::setw(6);
			outfile << std::right;
			outfile << b;
			outfile << std::endl;
		}
	}
	outfile.close();
	
}


// 设置k邻域值
void td::Segmentation::setKNearest(int k)
{
	this->k = k;
}

double td::Segmentation::getAverageError()
{
    return averageError;
}

const std::vector<td::PointCloud> &td::Segmentation::getIndices()
{
    return indices;
}

const std::vector<td::Plane>& td::Segmentation::getPlaneModels()
{
    return plane_models;
}

// //Multi-RANSAC进行分割
void td::Segmentation::multiRansac()
{
	indices.clear();
	double pro = 1.0;
	PointCloud mcloud = cloud;
	while (pro>0.2)
	{
		PlaneFitting fit;
		fit.setInputCloud(mcloud);
		bool state(false);
		state = selectMethod(fit, RANSAC);
		if (!state)
			break;
		//获取平面参数，及局内点
		Plane pl;
		pl = fit.getModel();
		PointCloud inliers;
        inliers = fit.getInliers();
        indices.push_back(inliers);
        plane_models.push_back(pl);
		mcloud = fit.getOutliers();
		pro = (double)mcloud.size() / cloud.size();
		double currentError = 0;
		for (size_t i = 0; i < inliers.size(); i++)
		{
			currentError += pow(pl.point2plane(inliers[i]), 2);
		}
		currentError /= inliers.size();
		currentError = sqrt(currentError);
		averageError += currentError;
	}
	averageError /= indices.size();
}


// 单模型拟合
void td::Segmentation::singleFitting(METHOD method, Plane& pl)
{
	PlaneFitting fit;
	fit.setInputCloud(cloud);
	bool state(false);
	state = selectMethod(fit, method);

	//获取平面参数，及局内点
//	Plane pl;
	pl = fit.getModel();
	PointCloud inliers;
	inliers = fit.getInliers();
	indices.push_back(inliers);
	double currentError = 0;
	for (size_t i = 0; i < inliers.size(); i++)
	{
		currentError += pow(pl.point2plane(inliers[i]), 2);
	}
	currentError /= inliers.size();
	currentError = sqrt(currentError);
	averageError += currentError;
	averageError /= indices.size();
}
