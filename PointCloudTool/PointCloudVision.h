
#include <QtWidgets/QMainWindow>

#include "ui_PointCloudVision.h"

#include <pcl/point_types.h>					//������������
#include <pcl/point_cloud.h>					//������
#include <pcl/visualization/pcl_visualizer.h>	//���ƿ��ӻ���
#include <vtkRenderWindow.h>					//vtk���ӻ�����
#include <pcl/common/common.h>					//���Ƽ�ֵ
#include "QHeightRampDlg.h"						//�߶���Ⱦ
#include "QOctreeDialog.h"
#include "QOctrTree_my.h"
												//kd-tree
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
#include <vector>
using namespace std;
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

	vtkOrientationMarkerWidget *axes_widget_member;

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

	//�˲����Ի���
	QOctreeDialog octreeDialog;
	//�˲����Ի����Դ��汾
	//�˲����Դ��汾
	QOctrTree_my myOctreeDialog1;
	//kd���Ի���

	//�˲���������
	QToolButton *toolButton;
	//��׼������;
	QToolButton *toolButton2;
	//״̬��
	QLabel *statusLabel;

	//�Ŵ����
	double maxLen;

	//���ѡ��ص�����
	void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);

public slots:
	//�򿪵���
	void on_action_open_triggered();

	//�������
	void on_action_preserve_triggered();

	//�����ӽ�
	void on_action_reset_triggered();

	//��ͼ
	void on_action_screenshot_triggered();

	//�������Ŵ�
	void on_action_magnify_triggered();

	//��������С
	void on_action_shrink_triggered();

	//����
	void on_action_topMove_triggered();

	//����
	void on_action_bottomMove_triggered();

	//����
	void on_action_leftMove_triggered();

	//����
	void on_action_rightMove_triggered();

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

	//PCA-ICP
	void on_action_icp_triggered();

	//SCALE-ICP
	void on_action_action_scale_icp_triggered();

	//�˲�

	//�����˲�
	void on_action_3_triggered();

	//ֱͨ�˲�
	void on_action_4_triggered();
	
	//ͳ���˲�
	void on_action_5_triggered();

	//���Ȳ����˲�
	void on_action_6_triggered();

	//�뾶�˲�
	void on_action_7_triggered();

	//���Ʒ�����
	void on_action_cloud_normal_vector_2_triggered();
	
	//���и߶���Ⱦ
	void setHeightRamp(int, double);

	//�˲���;
	void on_action_octree_triggered();

	//�˲����Դ���
	void on_action_myoctree1_triggered();

	//�˲�����������;
	void octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b);

	//�˲������������ņ�;
	void octree_vsearch_zz(double resolution, double x, double y, double z, int r, int g, int b,int flag);

	//�˲���������������������
	void searchPointByOctreeCreatedByDepth(int depth, int pointNum, double radius, int flag, double x, double y, double z);

	//����index�ж����������ƶ�
	void changeLocationOfObject(int index);

	//���ѡ��
	void on_action_pickPoints_triggered();

	//�˲���k��������;
	//void ksearch(float resolution, double x, double y, double z, int k);

	//����ͷ
	void drawArrow(PointT start, PointT end, string id, vector<int> RGB = { 255,0,0 });

private:
	Ui::PointCloudVisionClass ui;
	vector<int> pointIndex;//�ڽ�������
};
