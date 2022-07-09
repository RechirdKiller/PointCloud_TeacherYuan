#pragma once

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtWidgets/QMainWindow>

#include "ui_PointCloudVision.h"

#include <pcl/point_types.h>					//������������
#include <pcl/point_cloud.h>					//������
#include <pcl/visualization/pcl_visualizer.h>	//���ƿ��ӻ���
#include <vtkRenderWindow.h>					//vtk���ӻ�����
#include <pcl/common/common.h>					//���Ƽ�ֵ
#include "QHeightRampDlg.h"						//�߶���Ⱦ
#include <pcl/filters/passthrough.h>			//ֱͨ�˲�
#include <pcl/io/pcd_io.h>						//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>						//PCL��PLY��ʽ�ļ����������ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>		//���Ʋ鿴����ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/segmentation/region_growing.h>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <string>

//�������ı���
#pragma execution_character_set("utf-8")

//�������Ͷ���
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudVision : public QMainWindow
{
	Q_OBJECT
			
public:
	PointCloudVision(QWidget *parent = Q_NULLPTR);

	//��ʼ��
	void init();


private:

	//��ǰ�ĵ���
	PointCloudT::Ptr m_currentCloud;

	//�߶���Ⱦ�ĵ�����
	QList<PointCloudT::Ptr> m_heightCloudList;

	//���ӻ�������
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//�������꼫ֵ
	PointT p_min, p_max;

	//�߶���Ⱦ�Ի���
	QHeightRampDlg heightRampDlg;
	double maxLen;

public slots:
	//�򿪵���
	void on_action_open_triggered();

	//�����ӽ�
	void on_action_reset_triggered();

	//����ͼ
	void on_action_up_triggered();

	//ǰ��ͼ
	void on_action_front_triggered();

	//����ͼ
	void on_action_left_triggered();

	//����ͼ
	void on_action_back_triggered();

	//����ͼ
	void on_action_right_triggered();

	//����ͼ
	void on_action_bottom_triggered();

	//ǰ���
	void on_action_frontIso_triggered();

	//�����
	void on_action_backIso_triggered();

	//���õ�����ɫ
	void on_action_setColor_triggered();

	//���ø߶���Ⱦ
	void on_action_heightRamp_triggered();
	//��������
	void on_action_triangle_triggered();
	//��������ȡ
	void on_action_feature_triggered();
	//���������ָ��㷨
	void on_action_grow_triggered();
	//���и߶���Ⱦ
	void setHeightRamp(int, double);
private:
	Ui::PointCloudVisionClass ui;
};
