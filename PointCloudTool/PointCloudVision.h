
#include <QtWidgets/QMainWindow>

#include "ui_PointCloudVision.h"

#include <pcl/point_types.h>					//点云数据类型
#include <pcl/point_cloud.h>					//点云类
#include <pcl/visualization/pcl_visualizer.h>	//点云可视化类
#include <vtkRenderWindow.h>					//vtk可视化窗口
#include <pcl/common/common.h>					//点云极值
#include "QHeightRampDlg.h"						//高度渲染
#include "QOctreeDialog.h"						//八叉树
												//kd-tree
#include <pcl/filters/passthrough.h>			//直通滤波
#include <pcl/io/pcd_io.h>						//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>						//PCL的PLY格式文件的输入输出头文件
#include <pcl/visualization/cloud_viewer.h>		//点云查看窗口头文件
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


//设置中文编码
#pragma execution_character_set("utf-8")

//点云类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudVision : public QMainWindow
{
	Q_OBJECT
			
public:
	PointCloudVision(QWidget *parent = Q_NULLPTR);

	//初始化
	void init();


private:

	vtkOrientationMarkerWidget *axes_widget_member;

	//当前的点云
	PointCloudT::Ptr m_currentCloud;

	//高度渲染的点云列
	QList<PointCloudT::Ptr> m_heightCloudList;

	//可视化窗口类
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//点云坐标极值
	PointT p_min, p_max;

	//高度渲染对话框
	QHeightRampDlg heightRampDlg;

	//八叉树对话框
	QOctreeDialog octreeDialog;

	//kd树对话框


	//滤波工具栏；
	QToolButton *toolButton;
	//配准工具栏;
	QToolButton *toolButton2;
	//状态栏
	QLabel *statusLabel;

	//放大次数
	double maxLen;

public slots:
	//打开点云
	void on_action_open_triggered();

	//保存点云
	void on_action_preserve_triggered();

	//重设视角
	void on_action_reset_triggered();

	//截图
	void on_action_screenshot_triggered();

	//按比例放大
	void on_action_magnify_triggered();

	//按比例缩小
	void on_action_shrink_triggered();

	//上移
	void on_action_topMove_triggered();

	//下移
	void on_action_bottomMove_triggered();

	//左移
	void on_action_leftMove_triggered();

	//右移
	void on_action_rightMove_triggered();

	//俯视图
	void on_action_up_triggered();

	//前视图
	void on_action_front_triggered();

	//左视图
	void on_action_left_triggered();

	//后视图
	void on_action_back_triggered();

	//右视图
	void on_action_right_triggered();

	//底视图
	void on_action_bottom_triggered();

	//前轴测
	void on_action_frontIso_triggered();

	//后轴测
	void on_action_backIso_triggered();

	//设置点云颜色
	void on_action_setColor_triggered();

	//设置高度渲染
	void on_action_heightRamp_triggered();
	
	//三角网格化
	void on_action_triangle_triggered();
	
	//特征点提取
	void on_action_feature_triggered();
	
	//区域增长分割算法
	void on_action_grow_triggered();

	//PCA-ICP
	void on_action_icp_triggered();

	//SCALE-ICP
	void on_action_action_scale_icp_triggered();

	//滤波

	//体素滤波
	void on_action_3_triggered();

	//直通滤波
	void on_action_4_triggered();
	
	//统计滤波
	void on_action_5_triggered();

	//均匀采样滤波
	void on_action_6_triggered();

	//半径滤波
	void on_action_7_triggered();
	
	//进行高度渲染
	void setHeightRamp(int, double);

	//八叉树;
	void on_action_octree_triggered();

	//八叉树体素搜索;
	void octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b);

	//根据index判断上下左右移动
	void changeLocationOfObject(int index);

	//八叉树k近邻搜索;
	//void ksearch(float resolution, double x, double y, double z, int k);

private:
	Ui::PointCloudVisionClass ui;
};
