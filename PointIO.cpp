#include "PointIO.h"


td::PointIO::PointIO()
: cloud(0)
{
}


td::PointIO::PointIO(std::string path)
{
	std::ifstream infile;
	infile.open(path.c_str(), std::ios::in);
	if (!infile)
	{
		std::cerr << "文件有误" << std::endl;
		return;
	}
	std::string line, str;
	//暂时读入x,y,z形式的文件
	while (!infile.eof())
	{
		/*getline(infile,line);
		istringstream stream(line);

		stream>>tempx>>str>>tempy>>str>>tempz;*/

		Point point;
		infile >> point.x;
		infile >> point.y;
		infile >> point.z;
		cloud.push_back(point);
	}
	infile.close();
}

td::PointIO::~PointIO()
{
}


td::PointCloud td::PointIO::getPointCloud()
{
	return cloud;
}


// 读入文件
void td::PointIO::open(std::string path)
{
	std::ifstream infile;
	infile.open(path.c_str(), std::ios::in);
	if (!infile)
	{
		std::cerr << "文件有误" << std::endl;
		return;
	}
	std::string line, str;
	//暂时读入x,y,z形式的文件
	while (!infile.eof())
	{

		Point point;
		infile >> point.x;
		infile >> point.y;
		infile >> point.z;
		cloud.push_back(point);
	}
	infile.close();
}


void td::PointIO::save(std::string path)
{
	std::ofstream ofile;
	ofile.open(path.c_str(), std::ios::out);
	if (!ofile)
	{
		std::cerr << "文件有误" << std::endl;
		return;
	}
	//暂时读入x,y,z形式的文件
	for (size_t i = 0; i < cloud.size();++i)
	{

		ofile << cloud[i].x;
		ofile << cloud[i].y;
		ofile << cloud[i].z;
	}
	ofile.close();
}
