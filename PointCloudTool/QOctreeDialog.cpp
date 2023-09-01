#include "QOctreeDialog.h"


#include <QColorDialog>
#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include<iostream>
#include<vector>
#include<pcl/point_cloud.h>
 #include<pcl\visualization\pcl_visualizer.h>
#include "octtree_search_zz.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr creatSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>pointIndex);
void visualization(octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr creatSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>pointIndex);
void vsearch(octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
octree_my::octree createOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_point_size);

QOctreeDialog::QOctreeDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowModality(Qt::ApplicationModal);
	connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), ui.spinBox, SLOT(setValue(int)));
	connect(ui.spinBox, SIGNAL(valueChanged(int)), ui.horizontalSlider, SLOT(setValue(int)));

	connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)), ui.spinBox_2, SLOT(setValue(int)));
	connect(ui.spinBox_2, SIGNAL(valueChanged(int)), ui.horizontalSlider_2, SLOT(setValue(int)));
}

QOctreeDialog::~QOctreeDialog()
{
}

//体素近邻搜索
void QOctreeDialog::on_btn_vSearch_clicked()
{
	double resolution = ui.input_resolution->value();
	double x = ui.input_X->value();
	double y = ui.input_Y->value();
	double z = ui.input_Z->value();
	int orderNum = ui.spinBox_3->value();
	emit octree_vsearch(resolution, x, y, z, r, g, b, orderNum);
	close();
}



//k邻域搜索
void QOctreeDialog::on_btn_kSearch_clicked()
{
	double resolution = ui.input_resolution->value();
	double x = ui.input_X->value();
	double y = ui.input_Y->value();
	double z = ui.input_Z->value();
	int pointNum = ui.horizontalSlider->value();
	//int radiusAccount = ui.horizontalSlider_2->value();
	int orderNum = ui.spinBox_3->value();
	emit octree_kdtSearch(resolution, x, y, z, r, g, b, pointNum, orderNum);
	close();
}

//半径邻域搜索
void QOctreeDialog::on_btn_rSearch_clicked()
{
	double resolution = ui.input_resolution->value();
	double x = ui.input_X->value();
	double y = ui.input_Y->value();
	int radiusAccount = ui.horizontalSlider_2->value();
	double z = ui.input_Z->value();
	int orderNum = ui.spinBox_3->value();
	emit octree_radiusTreeSearsch(resolution, x, y, z, r, g, b,radiusAccount, orderNum);
	close();
}

//设置颜色
void QOctreeDialog::on_btn_color_clicked()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "设置搜索点颜色", QColorDialog::ShowAlphaChannel);
	color.getRgb(&r, &g, &b, nullptr);
}