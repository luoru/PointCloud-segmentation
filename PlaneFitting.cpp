#include "PlaneFitting.h"


td::PlaneFitting::PlaneFitting()
	: inliers_threshold(0)
{

}

td::PlaneFitting::PlaneFitting(const td::PointCloud& pc)
{
    cloud = pc;
}

td::PlaneFitting::~PlaneFitting()
{

}

//输入点云
void td::PlaneFitting::setInputCloud(const td::PointCloud& pc)
{
    cloud = pc;
}


//RANSAC进行拟合
bool td::PlaneFitting::computeByRANSAC(double threshold)
{
	srand((unsigned)time(NULL));
	if (cloud.size() <= 3)
		return false;
	//初始迭代次数
	int initial_iter = int(ceil(log10(0.01) / log10(1.0 - (double)model.needNum / cloud.size())));
	int iter(0);  //RANSAC循环变量
	int inliers_l(0), inliers_n(0);//上一次循环局内点数目，本次循环局内点数目 

	int randnum = model.needNum;
	int* nIndex=new int [randnum];    //随机点索引

	while (iter < initial_iter)
	{

		//抽取随机点
		for (int i = 0; i < randnum; i++)
		{
			nIndex[i] = rand() % cloud.size();
		}

		//判断抽取随机点是否重复
		bool allsame(true);
		for (int i = 1; i < randnum; i++)
		{
			allsame = allsame&&nIndex[i] == nIndex[i - 1];
		}
		if (allsame)
			continue;

		Plane pl;
		PointCloud randcloud;
		for (int i = 0; i < randnum; i++)
		{
			randcloud.push_back(cloud[nIndex[i]]);
		}
		pl.computeFromPoints(randcloud);

		double d2plane(0);
		inliers_n = 0;
		for (size_t i = 0; i < cloud.size(); ++i)
		{
			d2plane = pl.point2plane(cloud[i]);
			if (d2plane <= threshold)
			{
				inliers_n++;
			}
		}


		if (inliers_n > inliers_l)
		{
			inliers_l = inliers_n;
			initial_iter = int(ceil(log10(0.01) / log10(1.0 - pow((double)inliers_n / cloud.size(), 3))));//更新循环次数
			model.setPara(pl.getA(), pl.getB(), pl.getC(), pl.getD());
			iter = 0;
			continue;//进行下一次循环
		}
		iter++;
	}
	delete[]nIndex;
	inliers_threshold = threshold;
	return true;

}

//BAYSAC进行拟合
bool td::PlaneFitting::computeByBAYSAC(double threshold)
{
	srand((unsigned)time(NULL));
	if (cloud.size() <= 3)
		return false;

	std::vector<double> pro(cloud.size(), 0);//概率值，索引与点一一对应

	//初始迭代次数
	int initial_iter = int(ceil(log10(0.01) / log10(1.0 - (double)model.needNum / cloud.size())));
	int stas_num = int(0.1*(double)initial_iter);


	int randnum = model.needNum;
	int* nIndex = new int[randnum];    //随机点索引


	double para_percent = 0;//最优参数占总参数的比例

	//用于参数统计
	std::vector<double> ins, plnum; //记录平面参数
	std::vector<Plane> planes;

	//参数统计
	for (int iter = 0; iter < 6; )
	{

		//抽取随机点
		for (int i = 0; i < randnum; i++)
		{
			nIndex[i] = rand() % cloud.size();
		}

		//判断抽取随机点是否重复
		bool allsame(true);
		for (int i = 1; i < randnum; i++)
		{
			allsame = allsame&&nIndex[i] == nIndex[i - 1];
		}
		if (allsame)
			continue;

		Plane pl;
		PointCloud randcloud;
		for (int i = 0; i < randnum; i++)
		{
			randcloud.push_back(cloud[nIndex[i]]);
		}
		pl.computeFromPoints(randcloud);

		double d2plane(0);
		int inliers_n = 0;
		for (size_t i = 0; i < cloud.size(); ++i)
		{
			d2plane = pl.point2plane(cloud[i]);
			if (d2plane <= threshold)
			{
				inliers_n++;
			}
		}

		iter++;

		//每次计算的参数放入统计序列中
		planes.push_back(pl);
		ins.push_back(inliers_n);//该参数下局内点的个数
		plnum.push_back(1);


		//最优参数统计,当前参数和前面的参数进行比较,
		if (iter >= 1)
		{
			for (size_t i = 0; i < planes.size() - 1; ++i)
			{
				double para_dif = acos(pl.getA() * planes[i].getA() + pl.getB() * planes[i].getB()
					+ pl.getC() * planes[i].getC()) * 180 /3.14;
				para_dif = para_dif > 90 ? 180 - para_dif : para_dif;
				if (para_dif < 10)/*点到精度高角度小，点的精度差角度大*/
				{
					plnum[i]++;//统计第i套参数的数目
					if (inliers_n>ins[i])
					{
						planes[i].setPara(pl.getA(), pl.getB(), pl.getC(), pl.getD());
						ins[i] = inliers_n;
					}
					planes.pop_back();
					ins.pop_back();
					plnum.pop_back();
				}
			}
			para_percent = *max_element(plnum.begin(), plnum.end()) / (double)(iter);
		}
	}
	//贝叶斯过程
	std::vector<double>::iterator it_f = plnum.begin();
	std::vector<double>::iterator it_s = max_element(plnum.begin(), plnum.end());
	size_t para_best_index = it_s - it_f;

	//通过参数统计为点赋先验概率
	for (size_t i = 0; i < cloud.size(); i++)
	{
		double d2plane = planes[para_best_index].point2plane(cloud[i]);
		if (d2plane <= threshold)
		{
			pro[i] = 1 - d2plane / threshold;
		}
	}

	//重新开始循环，参数统计步骤已经结束
	int iter = 0;
	int inliers_l = 0;
	int inliers_n = 0;

	while (iter < initial_iter)
	{
		//找出概率值最高的三个点
		std::vector<double> temp = pro;
		for (int i = 0; i < randnum; i++)
		{
			std::vector<double>::iterator it;
			it = max_element(temp.begin(), temp.end());
			nIndex[i] = it - temp.begin();
			temp.erase(it);
		}
		temp.clear();

		Plane pl_;
		PointCloud randcloud_;
		for (int i = 0; i < randnum; i++)
		{
			randcloud_.push_back(cloud[nIndex[i]]);
		}
		pl_.computeFromPoints(randcloud_);


		inliers_n = 0;
		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			if (d2plane <= threshold)
			{
				inliers_n++;
			}
		}

		//更新假设点集先验概率
		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			if (d2plane <= threshold)
			{
				pro[i] = pro[i] * (double)inliers_n / cloud.size();
			}
		}


		//寻找局内点
		if (inliers_n > inliers_l)
		{
			inliers_l = inliers_n;
			initial_iter = int(ceil(log10(0.01) / log10(1.0 - pow((double)inliers_n / cloud.size(), 3))));//更新K值
			iter = 0;
			model.setPara(pl_.getA(), pl_.getB(), pl_.getC(), pl_.getD());
			continue;//进行下一次循环
		}
		iter++;
	}

	delete[]nIndex;
	inliers_threshold = threshold;
	return true;
}


bool td::PlaneFitting::computeByLMedS()
{
	srand((unsigned)time(NULL));
	if (cloud.size() <= 3)
		return false;

	int randnum = model.needNum;
	int* nIndex = new int[randnum];    //随机点索引
	//参数统计步骤已经结束
	int initial_iter = 100;
	int iter(0);  //RANSAC循环变量
	double min_median(1000), mid_deviation(0), bstd(0);//最小中值，本次循环局内点数目 


	//？循环次数和概率的更新还是需要阈值
	while (iter < initial_iter)
	{
		//找出概率值最高的三个点
		for (int i = 0; i < randnum; i++)
		{
			nIndex[i] = rand() % cloud.size();
		}

		//判断抽取随机点是否重复
		bool allsame(true);
		for (int i = 1; i < randnum; i++)
		{
			allsame = allsame&&nIndex[i] == nIndex[i - 1];
		}
		if (allsame)
			continue;

		Plane pl_;
		PointCloud randcloud_;
		for (int i = 0; i < randnum; i++)
		{
			randcloud_.push_back(cloud[nIndex[i]]);
		}

		pl_.computeFromPoints(randcloud_);

		std::vector<double> model_deviation_p(0);//统计偏差中值

		mid_deviation = 0;//中值

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			model_deviation_p.push_back(d2plane);
		}

		sort(model_deviation_p.begin(), model_deviation_p.end());
		if (model_deviation_p.size() % 2 == 0)
			mid_deviation = (model_deviation_p[model_deviation_p.size() / 2 - 1] +
			model_deviation_p[model_deviation_p.size() / 2]) / 2;
		else
			mid_deviation = model_deviation_p[model_deviation_p.size() / 2];

		//更新假设点集先验概率
		double std_ = 0;
		std_ = computeStd(pl_, cloud);

		//寻找最小中值，并计算模型参数
		if (mid_deviation < min_median)
		{
			min_median = mid_deviation;
			int inliers = 0;

			for (size_t i = 0; i < cloud.size(); ++i)
			{
				double d2plane = pl_.point2plane(cloud[i]);
				if (d2plane <= 2 * std_)
				{
					inliers++;
				}
			}
			bstd = std_;
			model.setPara(pl_.getA(), pl_.getB(), pl_.getC(), pl_.getD());
			initial_iter = int(ceil(log10(0.01) / log10(1.0 - pow((double)inliers / cloud.size(), 3))));
			iter = 0;
			continue;
		}
		iter++;
	}


	delete[]nIndex;
	inliers_threshold = 2 * bstd;
	return true;
}


//BAYLMEDS进行拟合
bool td::PlaneFitting::computeByBayLMedS()
{
	srand((unsigned)time(NULL));
	if (cloud.size() <= 3)
		return false;

	//初始迭代次数
	int initial_iter = int(ceil(log10(0.01) / log10(1.0 - (double)model.needNum / cloud.size())));
	int stas_num = int(0.1*(double)initial_iter);
	std::vector<double> pro(cloud.size(), 0);//概率值，索引与点一一对应
	int randnum = model.needNum;
	

	double para_percent = 0;//最优参数占总参数的比例

	//用于参数统计

	Plane stat_best_plane;
	double stat_min_penalty(std::numeric_limits<double>::max());
	
	//个人感觉进行参数统计不靠谱，通过寻找最优参数来确定先验概率
	for (int iter = 0; iter < 6;)
	{
		std::set<int> nIndex;//随机点索引
		for (int i = 0; i < randnum; i++)
		{
			int curIndex = rand() % cloud.size();
			nIndex.insert(curIndex);
		}
		//判断随机抽取的点是否重复
		if (nIndex.size() < randnum)
			continue;

		Plane pl;
		PointCloud randcloud;
		std::set<int>::const_iterator it_nIndex;
		for (it_nIndex = nIndex.begin(); it_nIndex != nIndex.end(); ++it_nIndex)
		{
			randcloud.push_back(cloud[*it_nIndex]);
		}
		pl.computeFromPoints(randcloud);

		std::vector<double> model_deviation(0);//统计偏差中值

		double cur_penalty = 0;
		for (size_t i = 0; i < cloud.size(); i++)
		{
			double d2plane = pl.point2plane(cloud[i]);
			model_deviation.push_back(d2plane);
		}

		sort(model_deviation.begin(), model_deviation.end());

		//获取中值
		if (model_deviation.size() % 2 == 0)
			cur_penalty = (model_deviation[model_deviation.size() / 2 - 1] + 
			model_deviation[model_deviation.size() / 2]) / 2;
		else
			cur_penalty = model_deviation[model_deviation.size() / 2];

		//更新统计中的最优中值
		if (cur_penalty < stat_min_penalty)
		{
			stat_min_penalty = cur_penalty;
			stat_best_plane = pl;
		}

		iter++;	
	}


	//贝叶斯过程
	/*std::vector<double>::iterator it_f = plnum.begin();
	std::vector<double>::iterator it_s = max_element(plnum.begin(), plnum.end());
	size_t para_best_index = it_s - it_f;*/

	//通过参数统计为点赋先验概率
	//计算标准差
	//double std = computeStd(planes[para_best_index], cloud);
	double th = computeMStd(stat_best_plane, cloud, stat_min_penalty);
	for (size_t i = 0; i < cloud.size(); i++)
	{
		double d2plane = stat_best_plane.point2plane(cloud[i]); 

		if (d2plane <= th)
		{
			pro[i] = 1 - d2plane / th;
		}
	}

	//参数统计步骤已经结束
	
	//double best_median(1000), mid_deviation(0), bstd(0);//最小中值，本次循环局内点数目 
	double best_penalty(std::numeric_limits<double>::max());
	double best_inliers(0);
	double best_th(th);

	//？循环次数和概率的更新还是需要阈值
	std::vector<int> g_index(randnum, 0);
	int iter(0);  //初始循环变量
	while (iter < initial_iter)
	{
		//找出概率值最高的三个点
		std::vector<int> cur_index(randnum, 0);
		std::vector<double> temp = pro;

		for (int i = 0; i < randnum; i++)
		{
			std::vector<double>::iterator it;
			it = max_element(temp.begin(), temp.end());
			int iIndex = it - temp.begin();
			cur_index[i] = iIndex;
			temp.erase(it);
		}

		bool goon = true;
		
		//排序
		std::sort(cur_index.begin(), cur_index.end());
		std::sort(g_index.begin(), g_index.end());

		for (int i = 0; i < randnum; i++)
		{
			goon = goon&&cur_index[i] == g_index[i];
		}

		if (goon&&iter>0)
			break;

		for (int i = 0; i < randnum; i++)
		{
			g_index[i] = cur_index[i];
		}

		Plane pl_;
		PointCloud randcloud_;
		for (int i = 0; i < randnum; i++)
		{
			randcloud_.push_back(cloud[cur_index[i]]);
		}

		pl_.computeFromPoints(randcloud_);

		std::vector<double> model_deviation_p(0);//统计偏差中值

		//mid_deviation = 0;//中值
		double cur_penalty(0);

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			model_deviation_p.push_back(d2plane);
		}

		sort(model_deviation_p.begin(), model_deviation_p.end());
		if (model_deviation_p.size() % 2 == 0)
			cur_penalty = (model_deviation_p[model_deviation_p.size() / 2 - 1] + 
			model_deviation_p[model_deviation_p.size() / 2]) / 2;
		else
			cur_penalty = model_deviation_p[model_deviation_p.size() / 2];

		//更新假设点集先验概率
		double k = 0;     //k为当前中值，局内点的个数
		double cur_th = 0;
		cur_th = computeMStd(pl_, cloud, cur_penalty);

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			if (d2plane <= best_th)
			{
				k = k + 1.0;
			}
		}

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			double d2plane = pl_.point2plane(cloud[i]);
			if (d2plane <= best_th)
			{
				pro[i] = pro[i] * k / cloud.size();
			}
		}

		//寻找最小中值，并计算模型参数
		if (cur_penalty < best_penalty)
		{
			best_penalty = cur_penalty;
			best_inliers = k;
			best_th = cur_th;
			/*for (size_t i = 0; i < cloud.size(); ++i)
			{
				double d2plane = pl_.point2plane(cloud[i]);
				if (d2plane <= kk * std_)
				{
					inliers++;
				}
			}
			bstd = std_;
			model.setPara(pl_.getA(), pl_.getB(), pl_.getC(), pl_.getD());*/
			model = pl_;
			//initial_iter = int(ceil(log10(0.01) / log10(1.0 - pow((double)best_inliers / cloud.size(), 3))));
			//iter = 0;
		}
		iter++;
	}

	inliers_threshold = best_th;
	return true;
}

// 返回平面参数
td::Plane td::PlaneFitting::getModel()
{
	return model;
}


// 返回局内点
td::PointCloud td::PlaneFitting::getInliers()
{
	PointCloud inliers;
	for (size_t i = 0; i < cloud.size(); ++i)
	{
		double d2plane = model.point2plane(cloud[i]);
		if (d2plane <= inliers_threshold)
		{
			inliers.push_back(cloud[i]);
		}
	}
	return inliers;
}


// 返回模型标准阈值
double td::PlaneFitting::getInlierThershold()
{
	return inliers_threshold;
}


// 返回局外点
td::PointCloud td::PlaneFitting::getOutliers()
{
	PointCloud outliers;
	for (size_t i = 0; i < cloud.size(); ++i)
	{
		double d2plane = model.point2plane(cloud[i]);
		if (d2plane > inliers_threshold)
		{
			outliers.push_back(cloud[i]);
		}
	}
	return outliers;
}


// 计算点到面的标准差
double td::PlaneFitting::computeStd(Plane& pl, PointCloud& cloud)
{
	double avg_d(0), avg_d2(0);
	for (size_t i = 0; i < cloud.size(); i++)
	{
		double d2plane = pl.point2plane(cloud[i]); 
		avg_d += d2plane;
		avg_d2 += pow(d2plane, 2.0);
	}
	double ptnum = (double)cloud.size();
	/*avg_d /= ptnum;
	avg_d2 /= ptnum;*/
	double std = sqrt(avg_d2/ptnum - pow(avg_d/ptnum, 2.0));
	return std;
}

// 最小中值对应的标准差,适用于最小中值中阈值的计算
// derta = 1.4856[1 + 5/(n-p)]/sqrt(Mj)
double td::PlaneFitting::computeMStd(Plane& pl, PointCloud& cloud, double penalty)
{
	double num = (double)cloud.size();
	double sigma = 1.4826 * (1 + 5.0 / (num - 3.0))*penalty;
	return 2.5*sigma;
}
