/*
*计算矩阵最小特征值对应特征向量
*罗中飞，2015.12
*/

#ifndef matrix_h
#define matrix_h

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>

namespace td
{
	class Matrix
	{
	public:
		Matrix();
		~Matrix();

		//计算最小特征值对应的特征向量
		static void minEigenvalueVector(Eigen::Matrix3d &_mat, Eigen::MatrixXd &_vectors)
		{
			Eigen::EigenSolver<Eigen::Matrix3d> es(_mat);
			Eigen::Matrix3d D = es.pseudoEigenvalueMatrix();
			Eigen::Matrix3d V = es.pseudoEigenvectors();
			int id = 0;
			double minv = LONG_MAX;
			for (int i = 0; i < _mat.rows(); ++i)
			{
				if (abs(D(i, i)) < minv)
				{
					minv = abs(D(i, i));
					id = i;
				}
			}
			_vectors << V(0, id), V(1, id), V(2, id);
		}
	};
}

#endif

