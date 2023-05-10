#include "stdafx.h"
#include <iostream>
#include <cmath>
#include <time.h>
#include <vector>
using namespace std;

//����M*M�������������������ֵ
class EigenSolution {

	int size = 3;
	int maxIter = 300;//��������ֵ��������

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

	//��A�ֽ�ΪQ��R��A(M*M) Q(M*M) R(M*M)
	vector<vector<vector<float>>> QR(vector<vector<float>> a)
	{
		vector<vector<vector<float>>> res;
		vector<vector<float>> q = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//Q����
		vector<vector<float>> r = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//R����

		for (int k = 0; k < size; k++)
		{
			float MOD = 0;
			for (int i = 0; i < size; i++)
			{
				MOD += a[i][k] * a[i][k];
			}
			r[k][k] = sqrt(MOD); // ����A��k�е�ģ�����ɹ�ʽ(4)����R�ĶԽ���Ԫ��||A:k||
			for (int i = 0; i < size; i++)
			{
				q[i][k] = a[i][k] / r[k][k]; // �ɹ�ʽ(2)��A��k�б�׼��֮���ΪQ�ĵ�k��
			}

			for (int i = k + 1; i < size; i++)
			{
				for (int j = 0; j < size; j++)
				{
					r[k][i] += a[j][i] * q[j][k]; // �ɹ�ʽ(4)������R�������ǲ���
				}
				for (int j = 0; j < size; j++)
				{
					a[j][i] -= r[k][i] * q[j][k]; // �ɹ�ʽ(1)���������A��ÿһ��
				}
			}
		}

		res.push_back(q);
		res.push_back(r);

		return res;
	}

public:

	//�������A������ֵ����������
	vector<vector<vector<float>>> computeEigenValueAndVector(vector<vector<float>> A) {

		vector<vector<float>> temp = A;//��¼ÿ�ε������ֽ����
		vector<vector<float>> Q = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//Q����
		vector<vector<float>> R = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };//R����
		vector<vector<float>> totalQ = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };//��¼Q�ĵ������
		vector<vector<vector<float>>> QRDec;//��¼QR�ֽ�Ľ��
		int count;//��¼��������
		vector<vector<float>> eigenValue;//����ֵ����
		vector<vector<float>> eigenVector;//������������
		vector<vector<vector<float>>> res;

		for (count = 0; count < maxIter; count++)
		{
			//�Ե�ǰ�����������QR�ֽ�
			QRDec = QR(temp);
			Q = QRDec[0];
			R = QRDec[1];
			//��ȡ��һ�ε������ֽ�ľ���
			temp = matrixMulty(R, Q);
			//��Q������е���
			totalQ = matrixMulty(totalQ, Q);
		}

		//��¼���ɵ�����ֵ
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