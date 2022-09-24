#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include<iostream>
#include<vector>
#include<pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
pcl::KdTreeFLANN<pcl::PointXYZ> createKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void ksearch(pcl::KdTreeFLANN<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance);
void rsearch(pcl::KdTreeFLANN<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, double radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance);
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	char strfilepath[256] = "rabbit.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> tree = createKdTree(cloud);//����Kd��
	int count = 2;
	pcl::PointXYZ searchPoint;//��ʼ�����㣻
	searchPoint.x = 0;
	searchPoint.y = 0;
	searchPoint.z = 0;
	//����ṹ��
	vector<int>pointIndex(count);//�ٽ����±ꣻ
	vector<float>pointSquaredDistance(count);//�ٽ�����룻
	//������
	ksearch(tree, searchPoint, count, pointIndex, pointSquaredDistance);//k��������
	double radius = 9.25;
	rsearch(tree, searchPoint, radius, count, pointIndex, pointSquaredDistance);//�뾶��������
	//cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����

	viewer.showCloud(cloud);
	system("pause");
	return 0;
}
pcl::KdTreeFLANN<pcl::PointXYZ> createKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	return tree;
}
void ksearch(pcl::KdTreeFLANN<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance)
{
	cout << "k��������\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			cout << "��" << i + 1 << "���ٽ��㣺\n"
				<<"x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<<"y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<<"z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< "squared distance=" << pointSquaredDistance[i] << "\n";
		}
	}
	else
	{
		PCL_ERROR("δ�ҵ��ٽ���");
	}
}
void rsearch(pcl::KdTreeFLANN<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, double radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance)
{
	cout << "�뾶��������\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size() ; i++)
		{
			if ((pointSquaredDistance[i] - radius) > 1e-16)
			{
				break;
			}
			cout << "��" << i + 1 << "���ٽ��㣺\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< "squared distance=" << pointSquaredDistance[i] << "\n";
		}
	}
	else
	{
		PCL_ERROR("δ�ҵ��ٽ���");
	}
}
