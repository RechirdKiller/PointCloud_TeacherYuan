#include "QOctreeDialog.h"
#include <QColorDialog>
#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include<iostream>
#include<vector>
#include<pcl/point_cloud.h>
#include<pcl\visualization\pcl_visualizer.h>
#include "octree_search_zz.h"

QOctreeDialog::QOctreeDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowModality(Qt::ApplicationModal);
}

QOctreeDialog::~QOctreeDialog()
{
}

void QOctreeDialog::on_btn_sSearch_clicked()
{
	
}

//���ؽ�������
void QOctreeDialog::on_btn_vSearch_clicked()
{
	double resolution = ui.input_resolution->value();
	double x = ui.input_X->value();
	double y = ui.input_Y->value();
	double z = ui.input_Z->value();
	emit octree_vsearch(resolution, x, y, z, r, g, b);
	close();
}

//k��������
void QOctreeDialog::on_btn_kSearch_clicked()
{
}

//�뾶��������
void QOctreeDialog::on_btn_rSearch_clicked()
{
}

//������ɫ
void QOctreeDialog::on_btn_color_clicked()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "������������ɫ", QColorDialog::ShowAlphaChannel);
	color.getRgb(&r, &g, &b, nullptr);
}

void vsearch(octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//��ʾ���ṹ;
	cout << "���ؽ�������" << "\n";
	if (tree.voxelSearch(searchPoint, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			cout << "��" << i + 1 << "���ٽ��㣺\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< endl;
		}
		/*-------------------------���ӻ�-------------------------------------------*/
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("���ӻ�����2"));
		viewer->setBackgroundColor(255, 255, 255);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_color(cloud, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, set_color, "Points");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Points");	//���õ�Ĵ�С

		pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud = creatSearchCloud(cloud, pointIndex);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_searchColor(searchCloud, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(searchCloud, set_searchColor, "searchPoints");

		tree.visualize(viewer, tree.getRoot(), cloud);

		viewer->spin();
		system("pause");


	}
	else
	{
		PCL_ERROR("δ�ҵ��ٽ���");
	}
}
