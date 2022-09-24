#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
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
	pcl::KdTreeFLANN<pcl::PointXYZ> tree = createKdTree(cloud);//创建Kd树
	int count = 2;
	pcl::PointXYZ searchPoint;//起始搜索点；
	searchPoint.x = 0;
	searchPoint.y = 0;
	searchPoint.z = 0;
	//储存结构；
	vector<int>pointIndex(count);//临近点下标；
	vector<float>pointSquaredDistance(count);//临近点距离；
	//搜索；
	ksearch(tree, searchPoint, count, pointIndex, pointSquaredDistance);//k近邻搜索
	double radius = 9.25;
	rsearch(tree, searchPoint, radius, count, pointIndex, pointSquaredDistance);//半径近邻搜索
	//cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象

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
	cout << "k近邻搜索\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			cout << "第" << i + 1 << "个临近点：\n"
				<<"x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<<"y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<<"z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< "squared distance=" << pointSquaredDistance[i] << "\n";
		}
	}
	else
	{
		PCL_ERROR("未找到临近点");
	}
}
void rsearch(pcl::KdTreeFLANN<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, double radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance)
{
	cout << "半径近邻搜索\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size() ; i++)
		{
			if ((pointSquaredDistance[i] - radius) > 1e-16)
			{
				break;
			}
			cout << "第" << i + 1 << "个临近点：\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< "squared distance=" << pointSquaredDistance[i] << "\n";
		}
	}
	else
	{
		PCL_ERROR("未找到临近点");
	}
}
