#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include<iostream>
#include<vector>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
using namespace std;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> createOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  float resolution);
void vsearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex);
//resolution�Ƿֱ��ʣ���С���سߴ磻
void ksearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance);
void rsearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, double radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance);
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	char strfilepath[256] = "rabbit.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����
	//�����˲���
	float resolution = 128.0f;//�ֱ��ʣ���С���سߴ磻
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree = createOctree(cloud, resolution);
	//��ʼ�����㣻
	pcl::PointXYZ searchPoint;
	searchPoint.x = 0;
	searchPoint.y = 0;
	searchPoint.z = 0;
	int count = 2;
	//����ṹ
	vector<int> pointIndex;
	vector<float>pointSquaredDistance(count);//�ٽ�����룻
	//����
	vsearch(tree, searchPoint, pointIndex);//���ؽ���������
	ksearch(tree, searchPoint, count, pointIndex, pointSquaredDistance);//k��������
	double radius = 9.25;
	rsearch(tree, searchPoint, radius, count, pointIndex, pointSquaredDistance);//�뾶��������
	viewer.showCloud(cloud);
	system("pause");
	return 0;
}
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> createOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution)
{
	pcl::octree::OctreePointCloudSearch < pcl::PointXYZ> tree(resolution);
	tree.setInputCloud(cloud);
	//����߽��
	tree.defineBoundingBox();
	tree.addPointsFromInputCloud();
	return tree;
}
void vsearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex)
{
	cout << "���ؽ�������";
	if (tree.voxelSearch(searchPoint, pointIndex))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			cout << "��" << i + 1 << "���ٽ��㣺\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< endl;
		}
	}
	else
	{
		PCL_ERROR("δ�ҵ��ٽ���");
	}
}
void ksearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance)
{
	cout << "k��������\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
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
void rsearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree, pcl::PointXYZ searchPoint, double radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance)
{
	cout << "�뾶��������\n";
	if (tree.nearestKSearch(searchPoint, count, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
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