#include "stdafx.h"
#include <iostream>
#include <cmath>
#include <time.h>
#include <vector>
using namespace std;

//计算M*M矩阵的特征向量和特征值
class EigenSolution {

	int size = 3;
	int maxIter = 300;//计算特征值最大迭代量

protected:

	// matrixA * matrixB
	vector<vector<float>> matrixMulty(vector<vector<float>> matrixA, vector<vector<float>> matrixB)
	{
		float temp;
		vector<float> tempRow;
		vector<vector<float>> tempC;
		int row_i, column_i;
		int i;

		for (row_i = 0; row_i < size; ++row_i)
		{
			for (column_i = 0; column_i < size; ++column_i)
			{
				temp = 0;
				for (i = 0; i < size; ++i) {

					temp += matrixA[row_i][i] * matrixB[i][column_i];
				}
				tempRow.push_back(temp);
			}
			tempC.push_back(tempRow);
			tempRow.clear();
		}

		return tempC;
	}

	//将A分解为Q和R（A(M*M) Q(M*M) R(M*M)
	vector<vector<vector<float>>> QR(vector<vector<float>> a)
	{
		vector<vector<vector<float>>> res;
		vector<vector<float>> q = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//Q矩阵
		vector<vector<float>> r = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//R矩阵

		for (int k = 0; k < size; k++)
		{
			float MOD = 0;
			for (int i = 0; i < size; i++)
			{
				MOD += a[i][k] * a[i][k];
			}
			r[k][k] = sqrt(MOD); // 计算A第k列的模长，由公式(4)等于R的对角线元素||A:k||
			for (int i = 0; i < size; i++)
			{
				q[i][k] = a[i][k] / r[k][k]; // 由公式(2)，A第k列标准化之后成为Q的第k列
			}

			for (int i = k + 1; i < size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					r[k][i] += a[j][i] * q[j][k]; // 由公式(4)，计算R的上三角部分
				}
				for (int j = 0; j < size; j++)
				{
					a[j][i] -= r[k][i] * q[j][k]; // 由公式(1)，计算更新A的每一列
				}
			}
		}

		res.push_back(q);
		res.push_back(r);

		return res;
	}

public:

	//计算矩阵A的特征值和特征向量
	vector<vector<vector<float>>> computeEigenValueAndVector(vector<vector<float>> A) {

		vector<vector<float>> temp = A;//记录每次迭代被分解矩阵
		vector<vector<float>> Q = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//Q矩阵
		vector<vector<float>> R = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//R矩阵
		vector<vector<float>> totalQ = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };//记录Q的迭代结果
		vector<vector<vector<float>>> QRDec;//记录QR分解的结果
		int count;//记录迭代次数
		vector<vector<float>> eigenValue;//特征值矩阵
		vector<vector<float>> eigenVector;//特征向量矩阵
		vector<vector<vector<float>>> res;

		for (count = 0; count < maxIter; count++)
		{
			//对当前迭代矩阵进行QR分解
			QRDec = QR(temp);
			Q = QRDec[0];
			R = QRDec[1];
			//获取下一次迭代被分解的矩阵
			temp = matrixMulty(R, Q);
			//对Q矩阵进行迭代
			totalQ = matrixMulty(totalQ, Q);
		}

		//记录生成的特征值
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++) {

				if (i != j) {

					temp[i][j] = 0;
				}
			}
		}
		eigenValue = temp;
		eigenVector = totalQ;
		res.push_back(eigenValue);
		res.push_back(eigenVector);
		return res;
	}
};