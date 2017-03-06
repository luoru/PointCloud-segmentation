/*
*分割拟合算法测试主程序
*罗中飞，2015.12
*/

#include <iostream>
#include <string>
#include <ctime>

#include "PointIO.h"
#include "Segmentation.h"

int main()
{
	std::cout << "------------------------分割拟合程序开始运行---------------------"<< std::endl;
	std::cout << "输入点云文件路径（不能带有空格）"<<std::endl;
	
	std::string path;
	std::cin >> path;

	td::PointIO ptIO;
	ptIO.open(path);
	td::Segmentation seg;
	seg.setInputCloud(ptIO.getPointCloud());

    std::cout << "k邻域值(根据实际点云密度进行选择)" << std::endl;
	int k;
	std::cin >> k;
	seg.setKNearest(k);

	/*std::cout << "邻域半径" << std::endl;
	double radius;
	std::cin >> radius;
	seg.setRadius(radius);*/

	std::cout << "平面拟合阈值" << std::endl;
	double threshold;
	std::cin >> threshold;
	seg.setThreshold(threshold);
	
//	std::cout << "------------------------输入拟合方法---------------------" << std::endl;
//	std::cout << "------------------------0：RANSAC---------------------" << std::endl;
//	std::cout << "------------------------1：BAYSAC---------------------" << std::endl;
//	std::cout << "------------------------2：LMEDS---------------------" << std::endl;
//	std::cout << "------------------------3：BAYLMEDS---------------------" << std::endl;
//	std::cout << "------------------------开始生长拟合---------------------" << std::endl;
//	int me;
//	std::cin >> me;

	clock_t start, end;
	start = clock();
    std::cout << "拟合中..." <<std::endl;
//    seg.regionGrow(td::Segmentation::RANSAC, k);
    seg.multiRansac();
    //seg.singleFitting(td::Segmentation::RANSAC);
	end = clock();
//    std::cout << "拟合时间：" << (double)(end - start) / CLOCKS_PER_SEC *1000.0 << "ms" << std::endl;
//    std::cout << "共拟合" << seg.getModelNum() << "个平面" << std::endl;
//    std::cout << "平均拟合误差为" << seg.getAverageError() << std::endl;


    std::cout << "拟合结束!" << std::endl;

//    std::cout << "平面最少点数" << std::endl;
//    int minN;
//    std::cin >> minN;
//    std::cout << "计算挡板中..." << std::endl;

//    td::Plane normalV;
//    seg.singleFitting(td::Segmentation::RANSAC, normalV);

//    double n1[3];
//    n1[0] = normalV.getA();
//    n1[1] = normalV.getB();
//    n1[2] = normalV.getC();
//    std::vector<td::Plane> pls;
//    pls = seg.getPlaneModels();
//    std::vector<td::PointCloud> indice = seg.getIndices();

//    int tagetNum = 0;
//    for(int i = 0; i < pls.size(); i++)
//    {

//        if(indice[i].size()< minN)
//            continue;
//        double n2[3];
//        n2[0] = pls[i].getA();
//        n2[1] = pls[i].getB();
//        n2[3] = pls[i].getC();
//        double theta(0);
//        theta = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];
//        if(theta < 1.0)
//            tagetNum++;
//    }
//    std::cout << "共有 " << tagetNum << "块挡板" << std::endl;

	std::cout << "输入存储点云文件路径（不能带有空格）" << std::endl;
	std::cin >> path;
	seg.outPut(path);
	return 0;
}
