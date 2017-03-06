/*
*模型基类
*罗中飞，2015.12
*/
#ifndef model_h
#define model_h

namespace td
{
	class Model
	{
	public:
		Model();
		~Model();

		// 模型拟合需要的点数
		int needNum;
	};
}
#endif

