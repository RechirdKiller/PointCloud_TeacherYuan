#include "PointCloudVision.h"
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QColorDialog>
#include <QToolButton>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/ModelCoefficients.h>			
#include <pcl/filters/project_inliers.h>	
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <qmainwindow.h>
#include "PCA_ICP.h"
#include "scale_ICP_v2.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "pcl/common/transforms.h"
#include <QScreen>
#include <QPixmap>
#include <QGuiApplication>
#include <QtWidgets/QApplication>
#include <iostream>
#include"octtree_search_zz.h"
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>
#include<pcl\features\principal_curvatures.h>


VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

using namespace std;
#pragma execution_character_set("utf-8")
QString Last_FileName=nullptr;
double dist_scale = 1;

//鼠标选点结构体
struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

PointCloudVision::PointCloudVision(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//初始化
	init();
	//dist_scale = 1;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr creatSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>pointIndex);
void visualization(pcl::visualization::PCLVisualizer::Ptr viewer,octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr creatSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>pointIndex);
void vsearch(pcl::visualization::PCLVisualizer::Ptr viewer,Ui::PointCloudVisionClass ui,octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
octree_my::octree createMyOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_point_size);
octree_my::octree createMyOctreeByDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int depth);
void ksearch(Ui::PointCloudVisionClass ui, pcl::visualization::PCLVisualizer::Ptr viewer, octree_my::octree tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void rsearch(Ui::PointCloudVisionClass ui, pcl::visualization::PCLVisualizer::Ptr viewer, octree_my::octree tree, pcl::PointXYZ searchPoint, float radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//获取两个点平行于坐标轴的最短距离
double getMinValue(PointT p1, PointT p2);

//获取两个点平行于坐标轴的最长距离
double getMaxValue(PointT p1, PointT p2);

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
//求两个点的中点
double* getCenterPoint(PointT p1, PointT p2);

//以中点为中心，三个维度的最小距离的0.3倍为坐标轴的标刻度，最大轴长度为大小设置坐标轴;
void setCoordinate(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud);
void outputInDebug(string info);
//显示坐标轴xyz（未完成）
void addOrientationMarkerWidgetAxesToview(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, vtkRenderWindowInteractor* interactor, double x, double y, double x_wide, double y_wide)
{

}

//求坐标轴原点
void getCenterPoint(PointCloudT::Ptr m_currentCloud, double* centerPoint)
{
	double sumOfX = 0;
	double sumOfY = 0;
	double sumOfZ = 0;
	for (auto p : m_currentCloud->points)
	{
		sumOfX += p.x;
		sumOfY += p.y;
		sumOfZ += p.z;
	}

	centerPoint[0] = sumOfX / m_currentCloud->points.size();
	centerPoint[1] = sumOfY / m_currentCloud->points.size();
	centerPoint[2] = sumOfZ / m_currentCloud->points.size();

}

//初始化
void PointCloudVision::init()
{
	//点云初始化
	m_currentCloud.reset(new PointCloudT);
	//可视化对象初始化
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	//添加坐标轴
	//vtkSmartPointer<vtkRenderWindow> renderer = vtkSmartPointer<vtkRenderWindow>::New();

	vtkSmartPointer<vtkRenderer> rendererVic;
	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	axes->SetAxisLabels(1);
	axes->SetVisibility(true);
	//rendererVic->AddActor(axes);
	//viewer->getRenderWindow()->AddRenderer(rendererVic);
	//viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor2D(axes);
	//设置VTK可视化窗口指针
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//ui.qvtkWidget->SetRenderWindow(renderer);
	//设置窗口交互，窗口可接受键盘等事件
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	viewer->setBackgroundColor((double)113/255, (double)110/255, (double)119/255);
	viewer->addOrientationMarkerWidgetAxes(viewer->getRenderWindow()->GetInteractor());
	
	//添加坐标轴                                
	viewer->addCoordinateSystem(0.125, 0);
	
	//槽函数
	//高度渲染
	connect(&heightRampDlg, SIGNAL(setHeightRamp(int, double)), this, SLOT(setHeightRamp(int, double)));
	//八叉树体素近邻搜索-调库
	connect(&octreeDialog, SIGNAL(octree_vsearch(double, double, double, double, int, int, int, int)), this, SLOT(octree_vsearch(double, double, double, double, int, int, int, int)));
	//octree_kdtSearch-调库 k邻近
	connect(&octreeDialog, SIGNAL(octree_kdtSearch(double, double, double, double, int, int, int, int, int)), this, SLOT(octree_kdtSearch(double, double, double, double, int, int, int, int, int)));
	//octree_treerdtree 半径邻搜索
	connect(&octreeDialog, SIGNAL(octree_radiusTreeSearsch(double, double, double, double, int, int, int,int, int)), this, SLOT(octree_radiusTreeSearsch(double, double, double, double, int, int, int, int, int)));

	//octree
	connect(&octreeDialog, SIGNAL(octree_vsearch_zz(double, double, double, double, int, int, int,int)), this, SLOT(octree_vsearch_zz(double, double, double, double, int, int, int,int)));
	//层数创建八叉树-自创张喆
	connect(&myOctreeDialog1, SIGNAL(searchPointByOctreeCreatedByDepth(int, int, double, int, double, double, double)), this, SLOT(searchPointByOctreeCreatedByDepth(int, int, double, int, double, double, double)));

	//区域分割
	connect(&son_dragon, SIGNAL(segmentation_calculate_by_choice(int , double , int , double , int, int)), this, SLOT(segmentation_calculate_by_choice(int, double, int, double, int, int)));

	//滤波
	
	connect(&filter_dialog, SIGNAL(filter_dialog_function(int)), this, SLOT(filter_dialog_function(int)));
	//设置滤波工具栏;
	toolButton = new QToolButton(this);
	//只显示图片;
	toolButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
	//按下按钮立即工作;
	toolButton->setPopupMode(QToolButton::InstantPopup);
	toolButton->setMenu(ui.menu_5);
	toolButton->setToolTip("滤波");
	toolButton->setIcon(QIcon(":/PointCloudVision/image/filtering.png"));
	ui.mainToolBar->addWidget(toolButton);

	//设置配准工具栏;
	toolButton2 = new QToolButton(this);
	//只显示图片;
	toolButton2->setToolButtonStyle(Qt::ToolButtonIconOnly);
	//按下按钮立即工作;
	toolButton2->setPopupMode(QToolButton::InstantPopup);
	toolButton2->setMenu(ui.menu_6);
	toolButton2->setToolTip("点云配准");
	toolButton2->setIcon(QIcon(":/PointCloudVision/image/registration.png"));
	ui.mainToolBar->addWidget(toolButton2);
	//状态信息局域栏；
	ui.label->setStyleSheet("background-color:rgb(240,240,240);color:rgb(255,0,0);");
	//显示区域
	//ui.textBrowser->setStyleSheet("background-color:rgb(230,231,232);color:rgb(255,0,0);");
	//边框颜色
	ui.textBrowser->setMaximumHeight(400);
	ui.label->setMaximumHeight(100);
	ui.frame->setMaximumHeight(500);
	ui.qvtkWidget->update();
}



//打开点云
void PointCloudVision::on_action_open_triggered()
{
	
	//在输出栏输出;
	ui.textBrowser->append("选择点云文件...");
	ui.textBrowser->update();
	
	//获取点云路径
	QString path = QFileDialog::getOpenFileName(this, "选择点云文件", ".//", "点云文件(*.txt *.pcd *.ply);;所有文件(*.*)");

	//读取点云数据
	QFile file(path);

	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		//清空点云
		m_currentCloud->clear();
		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		string Path = path.toStdString();
		Last_FileName = path;
		string suffixStr = Path.substr(Path.find_last_of('.') + 1);//获取文件后缀
		if (suffixStr == "pcd") {
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				//输出栏输出
				ui.textBrowser->append("警告：点云文件格式错误");
		  ui.textBrowser->update();
		  QApplication::processEvents();

				QMessageBox::warning(this, "警告", "点云文件格式错误！");
			}
			else {
				//输出栏输出打开成功
				QString outputString = "成功加载";
				outputString += path;
				ui.textBrowser->append(outputString);
				ui.textBrowser->update();
				QApplication::processEvents();

				if (!ui.statusBar == NULL) {
					ui.statusBar->removeWidget(ui.statusBar);
				}
				if (!statusLabel == NULL) {
					ui.statusBar->removeWidget(statusLabel);
				}
				statusLabel = new QLabel(path, this);
				ui.statusBar->addPermanentWidget(statusLabel); //现实永久信息
			}
		}
		else if (suffixStr == "ply") {
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				//输出栏输出
				ui.textBrowser->append("警告：点云文件格式错误");
		  ui.textBrowser->update();QApplication::processEvents();

				QMessageBox::warning(this, "警告", "点云文件格式错误！");
			}
			else {
				//输出栏输出打开成功
				QString outputString = "成功加载";
				outputString += path;
				ui.textBrowser->append(outputString);
		  ui.textBrowser->update();QApplication::processEvents();

				if (!statusLabel == NULL) {
					ui.statusBar->removeWidget(statusLabel);
				}
				statusLabel = new QLabel(path, this);
				ui.statusBar->addPermanentWidget(statusLabel); //现实永久信息
			}
		}
		
		else if(suffixStr == "txt") {
			int flag = 0;//1表示异常退出;
			while (!file.atEnd())
			{
				QByteArray line = file.readLine();
				QString str(line);
				QList<QString> strList = str.split(" ");

				if (strList.size() != 3)
				{
					strList.clear();
					strList = str.split("\t");
				}

				if (strList.size() != 3)
				{
					//输出栏输出
					ui.textBrowser->append("警告：点云文件格式错误");
			  ui.textBrowser->update();QApplication::processEvents();

					QMessageBox::warning(this, "警告", "点云文件格式错误！");
					flag = 1;
					break;
				}


				//点云赋值
				PointT point;
				point.x = strList.at(0).toDouble();
				point.y = strList.at(1).toDouble();
				point.z = strList.at(2).toDouble();

				m_currentCloud->push_back(point);
			}
			if (flag == 0) {
				//输出栏输出打开成功
				QString outputString = "成功加载";
				outputString += path;
				ui.textBrowser->append(outputString);
				ui.textBrowser->update();QApplication::processEvents();

				if (!statusLabel == NULL) {
					ui.statusBar->removeWidget(statusLabel);
				}
				statusLabel = new QLabel(path, this);
				ui.statusBar->addPermanentWidget(statusLabel); //现实永久信息
			}
		}
		
		//添加到窗口
		viewer->addPointCloud(m_currentCloud);
		//viewer->removeAllCoordinateSystems();
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//重设视角
		dist_scale = 1.0;
		viewer->resetCamera();
		outputInDebug(to_string(m_currentCloud->size()));
		//刷新窗口
		another_currentCloud = m_currentCloud;
		ui.qvtkWidget->update();
		on_action_reset_triggered();
	}
	else {
		ui.textBrowser->append("取消选择");
		ui.textBrowser->update();QApplication::processEvents();
		 
	}


	file.close();
	


}

LPCWSTR stringToLPCWSTR(string orig)
{
	size_t origsize = orig.length() + 1;
	const size_t newsize = 100;
	size_t convertedChars = 0;
	wchar_t* wcstring = new wchar_t[sizeof(wchar_t) * (orig.length() - 1)];
	mbstowcs_s(&convertedChars, wcstring, origsize, orig.c_str(), _TRUNCATE);
	return wcstring;
}

void outputInDebug(string info)
{
	LPCWSTR out = stringToLPCWSTR(info);
	OutputDebugString(out);
}

//保存点云
void PointCloudVision::on_action_preserve_triggered()
{
		QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply *.txt)"));
		//outputInDebug(filename.toStdString());
		//PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

		if (filename.isEmpty()) {
			ui.textBrowser->append("保存错误：未输入路径");
			ui.textBrowser->update();QApplication::processEvents();

			return;
		} 

		int return_status;
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *m_currentCloud);
		else if (filename.endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *m_currentCloud);
		else if (filename.endsWith(".txt", Qt::CaseInsensitive))
		{
			FILE* wc = fopen(filename.toStdString().data(), "w");
			int sizepcd = m_currentCloud->points.size();
			for (int i = 0; i < sizepcd; i++)
			{
				fprintf(wc, "%f\t%f\t%f\n", m_currentCloud->points[i].x, m_currentCloud->points[i].y, m_currentCloud->points[i].z);
			}

			fclose(wc);
		}
		else
		{
			filename.append(".ply");
			return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *m_currentCloud);
		}

		if (return_status != 0)
		{
			ui.textBrowser->append("写入错误");
	  ui.textBrowser->update();QApplication::processEvents();

			return;
		}
}

//计算矩阵特征向量和特征值
bool computeFeature(double * matrix, int size, double *featureVector, double *eigenValue, double dbEps = 0.01, int nJt = 300);
//求单个点的法向量
vector<double> computeNormal_(vector<PointT> points);


//点云法向量
void PointCloudVision::on_action_cloud_normal_vector_2_triggered()
{
	if (m_currentCloud->empty()) {

		ui.textBrowser->append("计算失败，未打开点云");
		return;
	}
	ui.textBrowser->append("开始计算法向量");

	PointT start;//法向量起始点
	PointT *end = new PointT();//法向量终点
	vector<PointT> proximityPoints;//临近点集合
	vector<int> proximityPointIndex;//临近点下标集合
	vector<float> pointSquaredDistance;
	vector<Eigen::MatrixXd> vectorP;//法向量坐标
	vector<vector<double>> normals;//存储所有点的法向量
	vector<double> temp;
	float size;//法向量长度
	PointT max, min;
	Eigen::Vector4f cloudCentroid;
	string s;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
	pcl::getMinMax3D(*m_currentCloud, min, max);
	size = sqrt(max.x*max.x + max.y*max.y + max.z*max.z) / 5;

	pcl::NormalEstimation<PointT, pcl::Normal> normObj;  //创建法向量计算对象  
	normObj.setInputCloud(m_currentCloud);                  //设置输入点云  
															//search::OrganizedNeighbor<PointXYZRGBA>::Ptr tree(new search::OrganizedNeighbor<PointXYZRGBA>());  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr KDtree(new pcl::search::KdTree<pcl::PointXYZ>());
	normObj.setSearchMethod(KDtree); //设置搜索方法  
	normObj.setKSearch(30);       //设置K邻域搜索  
	pcl::PointCloud<pcl::Normal>::Ptr Pnormals(new pcl::PointCloud<pcl::Normal>); //创建法向量点云  
	normObj.compute(*Pnormals); //计算法向量
	for (int i = 0; i < m_currentCloud->points.size(); i++) {
		start = m_currentCloud->points[i];
		if (((((start.x - cloudCentroid[0]) * (float)Pnormals->points[i].normal_x + (start.y - cloudCentroid[1]) * (float)Pnormals->points[i].normal_y + (start.z - cloudCentroid[2]) * (float)Pnormals->points[i].normal_z)) < 0))
		{
			end->x = start.x - (float)Pnormals->points[i].normal_x * size / 4;
			end->y = start.y - (float)Pnormals->points[i].normal_y * size / 4;
			end->z = start.z - (float)Pnormals->points[i].normal_z * size / 4;
		}
		else
		{
			end->x = start.x + (float)Pnormals->points[i].normal_x * size / 4;
			end->y = start.y + (float)Pnormals->points[i].normal_y * size / 4;
			end->z = start.z + (float)Pnormals->points[i].normal_z * size / 4;
		}

		/*s = "点" + to_string(i) + "法向量起点坐标为：" + to_string(start.x) + "," + to_string(start.y) + "," + to_string(start.z);
		ui.textBrowser->append(QString::fromStdString(s));
		s = "点" + to_string(i) + "法向量终点坐标为：" + to_string(end->x) + "," + to_string(end->y) + "," + to_string(end->z);
		ui.textBrowser->append(QString::fromStdString(s));*/

		//可视化
		string s = "arrow" + to_string(i);
		viewer->addLine<PointT>(start, *end, 255, 0, 0, s);
	}

	return;
}

//渐变色生成
vector<int> genGradientColor(double num, double width, vector<int> startColor, vector<int> endColor) {

	vector<int> color;

	double r = (endColor[0] - startColor[0]) / width;
	double g = (endColor[1] - startColor[1]) / width;
	double b = (endColor[2] - startColor[2]) / width;

	int R, G, B;

	R = startColor[0] + num*r;
	G = startColor[1] + num*g;
	B = startColor[2] + num*b;

	if (R > 255)R = 255;
	else
	{
		if (R < 0) R = 0;
	}

	if (G > 255)G = 255;
	else
	{
		if (G < 0)G = 0;
	}
	if (B > 255)B = 255;
	else
	{
		if (B < 0)B = 0;
	}
	color = { R,G,B };

	return color;
}

void PointCloudVision::on_action_normal_vector_library_version_triggered()
{
	if (m_currentCloud->empty()) {

		ui.textBrowser->append("曲率计算失败，未打开点云");
		return;
	}
	ui.textBrowser->append("开始计算曲率");


	vector<double> gaussCurvatureList;//整个点云高斯曲率列表
	vector<double> averageCurvatureList;//整个点云平均曲率列表
	pcl::NormalEstimation<PointT, pcl::Normal> normObj;  //创建法向量计算对象  
	normObj.setInputCloud(m_currentCloud);                  //设置输入点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr KDtree(new pcl::search::KdTree<pcl::PointXYZ>());
	normObj.setSearchMethod(KDtree); //设置搜索方法  
	normObj.setKSearch(30);       //设置K邻域搜索  
	pcl::PointCloud<pcl::Normal>::Ptr Pnormals(new pcl::PointCloud<pcl::Normal>); //创建法向量点云  
	normObj.compute(*Pnormals); //计算法向量

	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(m_currentCloud);
	pc.setInputNormals(Pnormals);
	pc.setSearchMethod(KDtree);
	//pc.setRadiusSearch(0.05);
	pc.setKSearch(30);
	pc.compute(*cloud_curvatures);

	for (int i = 0; i < cloud_curvatures->size(); i++) {
		//平均曲率
		averageCurvatureList.push_back(((*cloud_curvatures)[i].pc1 + (*cloud_curvatures)[i].pc2) / 2);
		//高斯曲率
		gaussCurvatureList.push_back((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2);
		ui.textBrowser->append(QString::fromStdString("(" + to_string(m_currentCloud->points[i].x) + "," + to_string(m_currentCloud->points[i].y) + "," + to_string(m_currentCloud->points[i].z) + ")" + "\t"
			+ to_string(((*cloud_curvatures)[i].pc1 + (*cloud_curvatures)[i].pc2) / 2) + "\t"
			+ to_string((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2)));
	}

	//可视化
	double minCurvature = averageCurvatureList.at(min_element(averageCurvatureList.begin(), averageCurvatureList.end()) - averageCurvatureList.begin());
	double maxCurvature = averageCurvatureList.at(max_element(averageCurvatureList.begin(), averageCurvatureList.end()) - averageCurvatureList.begin());
	ui.textBrowser->append(QString::fromStdString("最大曲率为:" + to_string(maxCurvature)));
	ui.textBrowser->append(QString::fromStdString("最小曲率为:" + to_string(minCurvature)));
	vector<int> color;//目标显示颜色
	vector<int> index;
	for (int i = 0; i < m_currentCloud->points.size(); i++) {

		index.clear();
		index.push_back(i);
		color = genGradientColor(averageCurvatureList[i] - minCurvature, maxCurvature - minCurvature, { 255,255,255 }, { 255,0,0 });

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = m_currentCloud->width;
		cloud->height = m_currentCloud->height;
		cloud->points.resize(1);
		cloud->points[0].x = m_currentCloud->points[i].x;
		cloud->points[0].y = m_currentCloud->points[i].y;
		cloud->points[0].z = m_currentCloud->points[i].z;
		ui.textBrowser->append(QString::fromStdString("点(" + to_string(m_currentCloud->points[i].x) + "," + to_string(m_currentCloud->points[i].y) + "," + to_string(m_currentCloud->points[i].z) + ")"));
		ui.textBrowser->append(QString::fromStdString("颜色为(" + to_string(color[0]) + "," + to_string(color[1]) + "," + to_string(color[2]) + ")"));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorP(cloud, color[0], color[1], color[2]);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, colorP, "averageCurvature" + to_string(i));
	}
	return;
	return;
}
//重设视角
void PointCloudVision::on_action_reset_triggered()
{
	ui.textBrowser->append("重设视角");
	ui.textBrowser->update();

	if (!m_currentCloud->empty())
	{
		viewer->removeAllShapes();
		pcl::visualization::PointCloudColorHandlerCustom<PointT> singelColor(m_currentCloud, 255, 255, 255);
		viewer->removeAllPointClouds();
		m_currentCloud = another_currentCloud;
		viewer->addPointCloud(m_currentCloud, singelColor, "myCloud", 0);
		viewer->resetCamera();
		setCoordinate(viewer, m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("重设成功");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("重设失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//截图
void PointCloudVision::on_action_screenshot_triggered()
{
	if (!m_currentCloud->empty()) {
		//完成截图操作
		QScreen *screen = QGuiApplication::primaryScreen();
		QPixmap pixmap = screen->grabWindow(ui.qvtkWidget->winId());

		//用户选取文件名
		QString filename = QFileDialog::getSaveFileName(this, tr("Save the image"), "", tr("Image (*.jpg *.png)"));

		if (filename.isEmpty()) {
			ui.textBrowser->append("保存错误：未输入路径");
	  ui.textBrowser->update();QApplication::processEvents();

			return;
		}

		bool finish = false;
		if (filename.endsWith(".png", Qt::CaseInsensitive))
			finish = pixmap.save(filename, "jpg");
		else if (filename.endsWith(".jpg", Qt::CaseInsensitive))
			finish = pixmap.save(filename, "jpg");
		else {
			filename.append(".jpg");
			finish = pixmap.save(filename, "jpg");
		}

		if (!finish)
		{
			ui.textBrowser->append("写入错误");
			ui.textBrowser->update(); QApplication::processEvents();

			return;
		}
		else {

			ui.textBrowser->append("写入成功，文件位置为：" + filename);
			ui.textBrowser->update(); QApplication::processEvents();

		}
		ui.textBrowser->append("截图已保存");
		ui.textBrowser->update(); QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("截图失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}



//按比例放大
void PointCloudVision::on_action_magnify_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.qvtkWidget->update();
		/*viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
		ui.qvtkWidget->update();
		ui.textBrowser->append("设置大小成功");
		*/
		if (dist_scale < 3) {
			dist_scale += 0.2;
		}else if(dist_scale>=3 && dist_scale<5){
			dist_scale += 0.5;
		}else{
			dist_scale = 5;
			ui.textBrowser->append("已到最大放大倍数");
			ui.textBrowser->update();QApplication::processEvents();
			return;
		}
		outputInDebug(to_string(dist_scale));
		pcl::PointCloud<PointT>::Ptr output(new PointCloudT);
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		transform.matrix()(0, 0) = dist_scale;
		transform.matrix()(1, 1) = dist_scale;
		transform.matrix()(2, 2) = dist_scale;

		pcl::transformPointCloud(*m_currentCloud, *output, transform);
		m_currentCloud = output;
		viewer->removeAllPointClouds();
		viewer->addPointCloud(output);
		ui.qvtkWidget->update();
		ui.textBrowser->append("设置大小成功");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("设置大小失败：未加载点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
	
}


//按比例缩小
void PointCloudVision::on_action_shrink_triggered()
{
	if (!m_currentCloud->empty()){
		ui.qvtkWidget->update();
		if (dist_scale > 3) {
			dist_scale -= 0.5;
		}
		else if (dist_scale >= 1 && dist_scale <= 3) {
			dist_scale -= 0.2;
		}
		else if (dist_scale >= 0.1&&dist_scale < 1)
		{
			dist_scale -= 0.1;
		}
		else{
			dist_scale = 0.1;
			ui.textBrowser->append("已到最小缩小倍数");
			ui.textBrowser->update(); 
			QApplication::processEvents();
			return;
		}
		outputInDebug(to_string(dist_scale));
		pcl::PointCloud<PointT>::Ptr output(new PointCloudT);
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		transform.matrix()(0, 0) = dist_scale;
		transform.matrix()(1, 1) = dist_scale;
		transform.matrix()(2, 2) = dist_scale;

		pcl::transformPointCloud(*m_currentCloud, *output, transform);
		pcl::copyPointCloud(*output, *m_currentCloud);
		//重新添加点云
		viewer->removeAllPointClouds();
		viewer->addPointCloud(m_currentCloud);
		ui.textBrowser->append("设置大小成功");
		ui.textBrowser->update(); 
		QApplication::processEvents();
		ui.qvtkWidget->update();

	} else {
		ui.textBrowser->append("设置大小失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();

	}
}

//鼠标选点回调函数
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removeAllPointClouds();
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	outputInDebug("x:" + to_string(current_point.x));
}

//鼠标选点
void PointCloudVision::on_action_pickPoints_triggered()
{
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	//viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	outputInDebug("Shift+click on three floor points, then press 'Q'..." );

	// Spin until 'Q' is pressed:
	//viewer->spin();
	//system("pause");
	ui.qvtkWidget->update();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);   //100??
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	outputInDebug(" 'Q'...");
}

//根据传入的index判断是否是上下左右移动
void PointCloudVision::changeLocationOfObject(int index)
{
	if (!m_currentCloud->empty()) {
		viewer->removeAllPointClouds();
		PointT p_min, p_max;
		double point[3];
		pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
		//获得整个坐标系的规模
		double scale = getMaxValue(p_max, p_min);
		outputInDebug(to_string(scale));

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		// 定义在y轴上的平移scale
		if (index == 0)
		{
			transform_2.translation() << 0.0, scale*0.2, 0.0;
		}
		else if (index == 1)
		{
			transform_2.translation() << 0.0, -scale*0.2, 0.0;
		}
		else if(index == 2)
		{
			transform_2.translation() << scale*0.2, 0.0, 0.0;
		}
		else
		{
			transform_2.translation() << -scale*0.2, 0.0, 0.0;
		}

		//开始变换
		pcl::transformPointCloud(*m_currentCloud, *cloud_transformed, transform_2);
		//重新赋值
		m_currentCloud = cloud_transformed;
		pcl::copyPointCloud(*cloud_transformed, *m_currentCloud);
		//重新添加点云
		viewer->removeAllPointClouds();
		viewer->addPointCloud(m_currentCloud);
		//重新计算坐标轴
		Eigen::Vector4f cloudCentroid;
		pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
		viewer->removeAllCoordinateSystems();
		viewer->addCoordinateSystem(scale*0.5, cloudCentroid[0], cloudCentroid[1], cloudCentroid[2], 0);
		ui.qvtkWidget->update();
	}
	else {
		ui.textBrowser->append("设置大小失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}

//上移
void PointCloudVision::on_action_topMove_triggered()
{
	changeLocationOfObject(0);
}

//下移
void PointCloudVision::on_action_bottomMove_triggered()
{
	changeLocationOfObject(1);
}

//左移
void PointCloudVision::on_action_leftMove_triggered()
{
	changeLocationOfObject(3);
}

//右移
void PointCloudVision::on_action_rightMove_triggered()
{
	changeLocationOfObject(2);
}

//改变视角
void changeViewOfObject(int index,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud,PointT p_min, PointT p_max, Ui::PointCloudVisionClass ui)
{
	double point[3];
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
	getCenterPoint(m_currentCloud, point);
	double scale = getMaxValue(p_max, p_min);
	outputInDebug(to_string(scale));
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
	viewer->removeAllCoordinateSystems();
	if (index == 0)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1] + scale * 5, cloudCentroid[2], 0, -0.5, 0, 0, 0, 0);
		ui.textBrowser->append("俯视图");
	}
	else if (index == 1)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1] - scale * 5, cloudCentroid[2], 0, 0.5, 0, 0, 1, 0);
		ui.textBrowser->append("底视图");
	}
	else if (index == 2)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1], cloudCentroid[2] + scale * 5, 0, 0, -0.5, 0, 1, 0);
		ui.textBrowser->append("前视图");
	}
	else if (index == 3)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1], cloudCentroid[2] - scale * 5, 0, 0, 0.5, 0, 1, 0);
		ui.textBrowser->append("后视图");
	}
	else if (index == 4)
	{
		viewer->setCameraPosition(cloudCentroid[0] - scale * 5, cloudCentroid[1], cloudCentroid[2], 0.5, 0, 0, 0, 1, 0);
		ui.textBrowser->append("左视图");
	}
	else if (index == 5)
	{
		viewer->setCameraPosition(cloudCentroid[0] + scale * 5, cloudCentroid[1], cloudCentroid[2], -0.5, 0, 0, 0, 1, 0);
		ui.textBrowser->append("右视图");
	}
	viewer->addCoordinateSystem(0.5*scale, cloudCentroid[0], cloudCentroid[1], cloudCentroid[2], 0);
	ui.qvtkWidget->update();
	ui.textBrowser->update(); QApplication::processEvents();
}

//俯视图
void PointCloudVision::on_action_up_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(0, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();

	}
}

//前视图
void PointCloudVision::on_action_front_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(2, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//左视图
void PointCloudVision::on_action_left_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(4, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
  ui.textBrowser->update();QApplication::processEvents();


	}
}

//后视图
void PointCloudVision::on_action_back_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(3, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
  ui.textBrowser->update();QApplication::processEvents();


	}
}

//右视图
void PointCloudVision::on_action_right_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(5, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//底视图
void PointCloudVision::on_action_bottom_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(1, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//前轴测
void PointCloudVision::on_action_frontIso_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, p_min.y - 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 1, 1, 0);
		ui.qvtkWidget->update();
		ui.textBrowser->append("前轴测");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}

//后轴测
void PointCloudVision::on_action_backIso_triggered()
{
	if (!m_currentCloud->empty()) {
		viewer->setCameraPosition(p_max.x + 2 * maxLen, p_max.y + 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), -1, -1, 0);
		ui.qvtkWidget->update();
		ui.textBrowser->append("后轴测");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("展示失败：未设置点云");
		ui.textBrowser->update();QApplication::processEvents();
	}

}

//设置点云颜色
void PointCloudVision::on_action_setColor_triggered()
{
	if (!m_currentCloud->empty()) {
		QColor color = QColorDialog::getColor(Qt::white, this, "设置点云颜色", QColorDialog::ShowAlphaChannel);
		viewer->removeAllPointClouds();
		pcl::visualization::PointCloudColorHandlerCustom<PointT> singelColor(m_currentCloud, color.red(), color.green(), color.blue());
		viewer->addPointCloud(m_currentCloud, singelColor, "myCloud", 0);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, color.alpha()*1.0 / 255, "myCloud");
		ui.qvtkWidget->update();
		ui.textBrowser->append("设置颜色成功");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("设置颜色失败：未设置点云");
		ui.textBrowser->update();
		QApplication::processEvents();
	}

}

//设置高度渲染
void PointCloudVision::on_action_heightRamp_triggered()
{
	if (!m_currentCloud->empty()) {
		heightRampDlg.show();
	}
	else {
		ui.textBrowser->append("设置高度渲染：未设置点云");
		ui.textBrowser->update();
		QApplication::processEvents();


	}
}

//进行高度渲染
void PointCloudVision::setHeightRamp(int dir, double height)
{
	
	ui.textBrowser->append("开始高度渲染");
	ui.textBrowser->update();
	QApplication::processEvents();
	//清空点云
	viewer->removeAllPointClouds();
	m_heightCloudList.clear();

	double min_range = 0;
	double max_range = 0;
	std::string field = "x";

	switch (dir)
	{
	case 0:
		min_range = p_min.x;
		max_range = p_max.x;
		field = "x";
		break;

	case 1:
		min_range = p_min.y;
		max_range = p_max.y;
		field = "y";
		break;

	case 2:
		min_range = p_min.z;
		max_range = p_max.z;
		field = "z";
		break;
	default:
		break;
	}

	for (double i = min_range - 1; i < max_range + height;)
	{
		PointCloudT::Ptr cloudTemp(new PointCloudT());

		pcl::PassThrough<PointT> pass;			//直通滤波器对象
		pass.setInputCloud(m_currentCloud);		//输入点云
		pass.setFilterFieldName(field);			//设置过滤字段
		pass.setFilterLimits(i, i + height);	//设置过滤范围
		pass.setFilterLimitsNegative(false);	//设置保留字段
		pass.filter(*cloudTemp);				//执行滤波

		i += height;

		m_heightCloudList.append(cloudTemp);
	}

	//分段渲染
	for (int j = 0; j < m_heightCloudList.size();j++)
	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> fieldColor(m_heightCloudList.at(j), field);
		std::string index = std::to_string(j);
		viewer->addPointCloud(m_heightCloudList.at(j), fieldColor, index);
	}
	ui.textBrowser->append("高度渲染成功");
	ui.textBrowser->update();


}

//八叉树;
void PointCloudVision::on_action_octree_triggered()
{
	if (!m_currentCloud->empty()) {
		octreeDialog.show();

		//构建八叉树;调库版

	}
	else {
		ui.textBrowser->append("八叉树搜索：未加载点云");
		ui.textBrowser->update(); 
		QApplication::processEvents();
	}
}

void PointCloudVision::on_action_myoctree1_triggered()
{
	if (!m_currentCloud->empty()) {
		myOctreeDialog1.show();
		//构建八叉树，自创版

	}
	else {
		ui.textBrowser->append("八叉树搜索：未加载点云");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}
//按层数构建八叉树
//八叉树搜索自创版
void PointCloudVision::searchPointByOctreeCreatedByDepth(int depth, int pointNum, double radius, int flag, double x, double y, double z)
{
	ui.textBrowser->append("开始搜索\n");
	ui.textBrowser->update();
	QApplication::processEvents();
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	octree_my::octree tree = createMyOctreeByDepth(m_currentCloud, depth);
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
	pcl::PointXYZ searchPoint;
	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;
	outputInDebug(to_string(cloudCentroid[0]));
	outputInDebug(to_string(cloudCentroid[1]));
	outputInDebug(to_string(cloudCentroid[2]));
	int count = pointNum;
	double radius_my = radius;

	vector<int> pointIndex;
	vector<float>pointSquaredDistance;
	visualization(viewer, tree, searchPoint, pointIndex, pointSquaredDistance, m_currentCloud);
	if (flag == 0)
	{
		vsearch(viewer, ui, tree, searchPoint, pointIndex, pointSquaredDistance, m_currentCloud);
	}
	else if (flag == 1)
	{
		 ksearch(ui,viewer, tree, searchPoint, count, pointIndex, pointSquaredDistance, m_currentCloud);
	}
	else
	{
		rsearch(ui,viewer,tree, searchPoint, radius_my, count, pointIndex, pointSquaredDistance, m_currentCloud);//半径近邻搜索
	}
}

//半径八叉树搜索_自创版
void rsearch(Ui::PointCloudVisionClass ui, pcl::visualization::PCLVisualizer::Ptr viewer, octree_my::octree tree, pcl::PointXYZ searchPoint, float radius, int count, vector<int>pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cout << "半径近邻搜索\n";
	if (tree.radiusSearch(searchPoint, radius, pointIndex, pointSquaredDistance))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			/*cout << "第" << i + 1 << "个临近点：\n"
			<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
			<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
			<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
			<< "squared distance=" << pointSquaredDistance[i] << "\n";*/
			string s = "第" + to_string(i + 1) + "个临近点:\n" + "x=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].x) + "\n" + "y=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].y) + "\n" + "z=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].z) + "\n";
			ui.textBrowser->append(QString::fromStdString(s));
			ui.textBrowser->update();
			QApplication::processEvents();
		}
		/*-------------------------可视化-------------------------------------------*/
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_color(cloud, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, set_color, "Points");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Points");	//设置点的大小
		tree.visualize(viewer, tree.getRoot(), cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud = creatSearchCloud(cloud, pointIndex);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_searchColor(searchCloud, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(searchCloud, set_searchColor, "searchPoints");
	}
	else
	{
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}


//构建八叉树；
//输入：搜索点云、分辨率
//输出：八叉树；
pcl::octree::OctreePointCloudSearch<PointT> createOctree(pcl::PointCloud<PointT>::Ptr cloud, double resolution)
{
	pcl::octree::OctreePointCloudSearch <PointT> tree(resolution);
	tree.setInputCloud(cloud);
	//定义边界框
	tree.defineBoundingBox();
	tree.addPointsFromInputCloud();
	return tree;
}
//八叉树体素搜索张喆(自创版)
//输入：像素；搜索点坐标;
void PointCloudVision::octree_vsearch_zz(double resolution, double x, double y, double z, int r, int g, int b,int flag)
{
	ui.textBrowser->append("开始搜索\n");
	ui.textBrowser->update();
	QApplication::processEvents();
	viewer->removeAllShapes();
	int depth = 8;
	int max_point_size = resolution;
	viewer->removeAllPointClouds();

	//构建八叉树;
	octree_my::octree tree = createMyOctree(m_currentCloud, max_point_size);
	pcl::PointXYZ searchPoint;
	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;
	int count = 100;
	int radius_my = 0.05;
	vector<int> pointIndex;
	vector<float>pointSquaredDistance;
	visualization(viewer, tree, searchPoint, pointIndex,pointSquaredDistance,m_currentCloud);
	if (flag == 0)
	{
		vsearch(viewer,ui,tree, searchPoint, pointIndex, pointSquaredDistance, m_currentCloud);
	}
	else if (flag == 1)
	{
		ksearch(ui, viewer, tree, searchPoint, count, pointIndex, pointSquaredDistance, m_currentCloud);
	}
	else
	{
		rsearch(ui, viewer, tree, searchPoint, radius_my, count, pointIndex, pointSquaredDistance, m_currentCloud);//半径近邻搜索
	}
	
}

void visualization(pcl::visualization::PCLVisualizer::Ptr viewer,octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, set_color, "Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Points");	//设置点的大小
	tree.visualize(viewer, tree.getRoot(), cloud);
}

//自创版K临近搜索具体实现
void ksearch(Ui::PointCloudVisionClass ui,pcl::visualization::PCLVisualizer::Ptr viewer, octree_my::octree tree, pcl::PointXYZ searchPoint, int count, vector<int>pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	cout << "k近邻搜索\n";
	if (tree.KSearch(searchPoint, pointIndex, pointSquaredDistance, count))
	{
		for (int i = 0; i < pointIndex.size(); i++)
		{
			/*cout << "第" << i + 1 << "个临近点：\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< "squared distance=" << pointSquaredDistance[i] << "\n";*/
			string s = "第" + to_string(i + 1) + "个临近点:\n" + "x=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].x) + "\n" + "y=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].y) + "\n" + "z=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].z) + "\n";
			ui.textBrowser->append(QString::fromStdString(s));
			ui.textBrowser->update();
			QApplication::processEvents();
		}
		/*-------------------------可视化-------------------------------------------*/
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("可视化窗口3"));
		//viewer->setBackgroundColor(255, 255, 255);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_color(cloud, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, set_color, "Points");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Points");	//设置点的大小
		tree.visualize(viewer, tree.getRoot(), cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud = creatSearchCloud(cloud, pointIndex);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_searchColor(searchCloud, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(searchCloud, set_searchColor, "searchPoints");
	}
	else
	{
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}

//自创版体素搜索具体实现
void vsearch(pcl::visualization::PCLVisualizer::Ptr viewer,Ui::PointCloudVisionClass ui,octree_my::octree tree, pcl::PointXYZ searchPoint, std::vector<int> pointIndex, vector<float>pointSquaredDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr m_currentCloud)
{
	if (tree.voxelSearch(searchPoint, pointIndex, pointSquaredDistance))
	{

		for (int i = 0; i < pointIndex.size(); i++)
		{

			string s = "第" + to_string(i + 1) + "个临近点:\n" + "x=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].x) + "\n" + "y=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].y) + "\n" + "z=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].z) + "\n";
			/*cout << "第" << i + 1 << "个临近点：\n"
			<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
			<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
			<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
			<< endl;*/
			ui.textBrowser->append(QString::fromStdString(s));
			ui.textBrowser->update();
			QApplication::processEvents();
		}
		/*-------------------------可视化-------------------------------------------*/
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_color(m_currentCloud, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(m_currentCloud, set_color);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Points");	//设置点的大小
		//添加到窗口
		pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud = creatSearchCloud(m_currentCloud, pointIndex);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> set_searchColor(searchCloud, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(searchCloud, set_searchColor, "searchPoints");

		tree.visualize(viewer, tree.getRoot(), m_currentCloud);
		ui.qvtkWidget->update();
	}
	else
	{
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr creatSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>pointIndex)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud(new pcl::PointCloud<pcl::PointXYZ>);
	searchCloud->width = cloud->width;
	searchCloud->height = cloud->height;
	searchCloud->points.resize(cloud->points.size());
	for (int i = 0; i < pointIndex.size(); i++) {
		searchCloud->points[i].x = cloud->points[pointIndex[i]].x;
		searchCloud->points[i].y = cloud->points[pointIndex[i]].y;
		searchCloud->points[i].z = cloud->points[pointIndex[i]].z;
	}
	return searchCloud;
}

octree_my::octree createMyOctreeByDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int depth)
{
	octree_my::octree tree = octree_my::octree::octree();
	tree.CreatTree_depth(cloud, depth);
	//tree.CreatTree_depth(cloud, depth);
	//定义边界框
	return tree;
}

octree_my::octree createMyOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_point_size)
{
	octree_my::octree tree = octree_my::octree::octree();
	tree.CreatTree_max_pointSize(cloud, max_point_size);
	//tree.CreatTree_depth(cloud, depth);
	//定义边界框
	return tree;
}

//八叉树k邻近搜索-调库
//输入：搜索点坐标，
void PointCloudVision::octree_kdtSearch(double resolution, double x, double y, double z, int r, int g, int b, int pointNum, int orderNum)
{
	ui.textBrowser->append("开始搜索\n");
	ui.textBrowser->update();
	QApplication::processEvents();

	viewer->removeAllPointClouds();
	pcl::KdTreeFLANN<pcl::PointXYZ> myKdTree;
	myKdTree.setInputCloud(m_currentCloud);
	pcl::PointXYZ searchPoint;
	if (orderNum == 0 || orderNum > m_currentCloud->points.size())
	{
		searchPoint.x = x;
		searchPoint.y = y;
		searchPoint.z = z;
	}
	else
	{
		searchPoint.x = m_currentCloud->points[orderNum].x;
		searchPoint.y = m_currentCloud->points[orderNum].y;
		searchPoint.z = m_currentCloud->points[orderNum].z;
	}

	int pointNum_K = pointNum;
	vector<int> pointIdxNKNSearch(pointNum_K);
	vector<float> pointNKNSquaredDistance(pointNum_K);
	string s = "K nearest neighbor search at (" + to_string(searchPoint.x) + " " + to_string(searchPoint.y) + " " + to_string(searchPoint.z) + " )with K = " + to_string(pointNum_K);
	ui.textBrowser->append(QString::fromStdString(s));

	// kdtree.nearestKSearch函数搜索距离索引点最近的K个点，搜到后返回搜到的点数。
	int res = myKdTree.nearestKSearch(searchPoint, pointNum_K, pointIdxNKNSearch, pointNKNSquaredDistance);
	string ss = "nearestKSearch返回值为：" + to_string(res);
	ui.textBrowser->append(QString::fromStdString(ss));

	if (myKdTree.nearestKSearch(searchPoint, pointNum_K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		// 打印输出。
		for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
		{
			string sss = " " + to_string(m_currentCloud->points[pointIdxNKNSearch[i]].x) + " " + to_string(m_currentCloud->points[pointIdxNKNSearch[i]].y) + " " + to_string(m_currentCloud->points[pointIdxNKNSearch[i]].z) + " "+  "(squared distance:"
				+ to_string(pointNKNSquaredDistance[i]) + ")";
			ui.textBrowser->append(QString::fromStdString(sss));
		}

		pcl::PointCloud<PointT>::Ptr searchCloud(new pcl::PointCloud<PointT>);
		searchCloud->width = m_currentCloud->width;
		searchCloud->height = m_currentCloud->height;
		searchCloud->points.resize(m_currentCloud->points.size());
		for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
			searchCloud->points[i].x = m_currentCloud->points[pointIdxNKNSearch[i]].x;
			searchCloud->points[i].y = m_currentCloud->points[pointIdxNKNSearch[i]].y;
			searchCloud->points[i].z = m_currentCloud->points[pointIdxNKNSearch[i]].z;
		}

		//添加到窗口
		viewer->addPointCloud(m_currentCloud);
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> set_searchColor(searchCloud, r, g, b);
		viewer->addPointCloud<PointT>(searchCloud, set_searchColor, "searchPoints");
		ui.qvtkWidget->update();
	}
	else
	{
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}

//八叉树体素搜索-调库
//输入：像素；搜索点坐标;
void PointCloudVision::octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b, int orderNum)
{
	ui.textBrowser->append("开始搜索\n");
	ui.textBrowser->update();
	QApplication::processEvents();

	viewer->removeAllPointClouds();
	//构建八叉树;
	pcl::octree::OctreePointCloudSearch<PointT> tree = createOctree(m_currentCloud, resolution);
	//定义搜索点
	//起始搜索点；
	PointT searchPoint;
	if (orderNum == 0 || orderNum > m_currentCloud->points.size())
	{
		searchPoint.x = x;
		searchPoint.y = y;
		searchPoint.z = z;
	}
	else
	{
		searchPoint.x = m_currentCloud->points[orderNum].x;
		searchPoint.y = m_currentCloud->points[orderNum].y;
		searchPoint.z = m_currentCloud->points[orderNum].z;
	}
	//清空点下标；
	std::vector<int> pointIndex;


	//搜索;
	if (tree.voxelSearch(searchPoint, pointIndex))
	{
	
		for (int i = 0; i < pointIndex.size(); i++)
		{
			
			string s = "第" + to_string(i + 1) + "个临近点:\n" + "x=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].x) + "\n" + "y=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].y) + "\n" + "z=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].z) + "\n";
			/*cout << "第" << i + 1 << "个临近点：\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< endl;*/
			ui.textBrowser->append(QString::fromStdString(s));
			ui.textBrowser->update(); 
			QApplication::processEvents();
		} 
		/*-------------------------可视化-------------------------------------------*/
		//为查找到点创建新点云;
		pcl::PointCloud<PointT>::Ptr searchCloud(new pcl::PointCloud<PointT>);
		searchCloud->width = m_currentCloud->width;
		searchCloud->height = m_currentCloud->height;
		searchCloud->points.resize(m_currentCloud->points.size());
		for (int i = 0; i < pointIndex.size(); i++) {
			searchCloud->points[i].x = m_currentCloud->points[pointIndex[i]].x;
			searchCloud->points[i].y = m_currentCloud->points[pointIndex[i]].y;
			searchCloud->points[i].z = m_currentCloud->points[pointIndex[i]].z;
		}

		//添加到窗口
		viewer->addPointCloud(m_currentCloud);

		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> set_searchColor(searchCloud, r, g, b);
		viewer->addPointCloud<PointT>(searchCloud, set_searchColor, "searchPoints");
		ui.qvtkWidget->update();

	}
	else
	{
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}

//八叉树半径邻搜索-调库版
void PointCloudVision::octree_radiusTreeSearsch(double resolution, double x, double y, double z, int r, int g, int b,int radiusAccount, int orderNum)
{
	ui.textBrowser->append("开始搜索\n");
	ui.textBrowser->update();
	QApplication::processEvents();
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
	maxLen = getMaxValue(p_max, p_min);

	//outputInDebug(to_string(maxLen) + "************");
	viewer->removeAllPointClouds();
	pcl::KdTreeFLANN<pcl::PointXYZ> myKdTree;
	myKdTree.setInputCloud(m_currentCloud);
	pcl::PointXYZ searchPoint;
	if (orderNum == 0 || orderNum > m_currentCloud->points.size())
	{
		searchPoint.x = x;
		searchPoint.y = y;
		searchPoint.z = z;
	}
	else
	{
		searchPoint.x = m_currentCloud->points[orderNum].x;
		searchPoint.y = m_currentCloud->points[orderNum].y;
		searchPoint.z = m_currentCloud->points[orderNum].z;
	}
	vector<int> pointIdxRadiusSearch;  // 存储近邻索引。
	vector<float> pointRadiusSquaredDistance;   // 存储近邻对应的平均距离。

	double radius = maxLen*((double)radiusAccount/100);
	int res = myKdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	string s = "R半径搜索方法：kdtree.radiusSearch返回值为：" + to_string(res);
	ui.textBrowser->append(QString::fromStdString(s));


	if (res > 0)
	{
		// 打印输出。
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
		{
			string sss = " " + to_string(m_currentCloud->points[pointIdxRadiusSearch[i]].x) + " " + to_string(m_currentCloud->points[pointIdxRadiusSearch[i]].y) + " " + to_string(m_currentCloud->points[pointIdxRadiusSearch[i]].z) + " " + "(squared distance:"
				+ to_string(pointRadiusSquaredDistance[i]) + ")";
			ui.textBrowser->append(QString::fromStdString(sss));
		}

		pcl::PointCloud<PointT>::Ptr searchCloud(new pcl::PointCloud<PointT>);
		searchCloud->width = m_currentCloud->width;
		searchCloud->height = m_currentCloud->height;
		searchCloud->points.resize(m_currentCloud->points.size());
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
			searchCloud->points[i].x = m_currentCloud->points[pointIdxRadiusSearch[i]].x;
			searchCloud->points[i].y = m_currentCloud->points[pointIdxRadiusSearch[i]].y;
			searchCloud->points[i].z = m_currentCloud->points[pointIdxRadiusSearch[i]].z;
		}

		//添加到窗口
		viewer->addPointCloud(m_currentCloud);
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> set_searchColor(searchCloud, r, g, b);
		viewer->addPointCloud<PointT>(searchCloud, set_searchColor, "searchPoints");
		ui.qvtkWidget->update();
		ui.textBrowser->update();
		QApplication::processEvents();
	}
	else
	{
		viewer->addPointCloud(m_currentCloud);
		ui.textBrowser->append("未搜索到点");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}


//三角网格化;调库
void PointCloudVision::on_action_triangle_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始三角网格化");
		ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();
		//法线估计对象
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		//存储估计的法线
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		//定义kd树指针
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(m_currentCloud);
		n.setInputCloud(m_currentCloud);
		n.setSearchMethod(tree);
		n.setKSearch(20);
		//估计法线存储到其中
		n.compute(*normals);//Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
		//链接字段
		pcl::concatenateFields(*m_currentCloud, *normals, *cloud_width_normals);

		//定义搜索树对象
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//点云构建搜索树
		tree2->setInputCloud(cloud_width_normals);

		//定义三角化对象
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		//存储最终三角化的网络模型
		pcl::PolygonMesh triangles;//设置连接点之间的最大距离，（即是三角形最大边长）
		gp3.setSearchRadius(200.0f);
		//设置各种参数值
		gp3.setMu(2.5f);
		gp3.setMaximumNearestNeighbors(100);
		gp3.setMaximumSurfaceAngle(M_PI_4);
		gp3.setMinimumAngle(M_PI / 18);
		gp3.setMaximumAngle(2 * M_PI / 3);
		gp3.setNormalConsistency(false);

		//设置搜索方法和输入点云
		gp3.setInputCloud(cloud_width_normals);
		gp3.setSearchMethod(tree2);

		//执行重构，结果保存在triangles中
		gp3.reconstruct(triangles);
		viewer->addPolygonMesh(triangles, "my");

		//setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//重设视角
		viewer->resetCamera();

		//刷新窗口
		ui.qvtkWidget->update();
		ui.textBrowser->append("三角网格化成功");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("三角网格化失败：未加载点云");
		ui.textBrowser->update();QApplication::processEvents();

	}
	
}
//特征点提取;
void PointCloudVision::on_action_feature_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始特征点提取");
		ui.textBrowser->update();QApplication::processEvents();
		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		float angular_resolution = 0.5f;
		float support_size = 0.2f;
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		bool rotation_invariant = true;
		angular_resolution = pcl::deg2rad(angular_resolution);

		//打开一个磁盘中的.pcd文件  但是如果没有指定就会自动生成
		pcl::PointCloud<pcl::PointXYZ>::Ptr    point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

		setUnseenToMaxRange = true;

		int M = m_currentCloud->points.size();
		for (int i = 0; i < M; i++)
		{
			pcl::PointXYZ p;
			p.x = m_currentCloud->points[i].x;
			p.y = m_currentCloud->points[i].y;
			p.z = m_currentCloud->points[i].z;
			point_cloud.points.push_back(p);
		}
		point_cloud.width = 1;
		point_cloud.height = M;
		//从点云中建立生成深度图
		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
			scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		range_image.integrateFarRanges(far_ranges);
		if (setUnseenToMaxRange)
			range_image.setUnseenToMaxRange();

		//打开3D viewer并加入点云
		//pcl::visualization::PCLVisualizer viewer("3D Viewer");
		//viewer.setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
		viewer->addPointCloud(range_image_ptr, range_image_color_handler, "range image");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		viewer->initCameraParameters();
		//viewer->resetCamera();
		setViewerPose(*viewer, range_image.getTransformationToWorldSystem());
		//提取NARF特征
		pcl::RangeImageBorderExtractor range_image_border_extractor;    //申明深度图边缘提取器
		pcl::NarfKeypoint narf_keypoint_detector;                       //narf_keypoint_detector为点云对象

		narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage(&range_image);
		narf_keypoint_detector.getParameters().support_size = support_size;    //获得特征提取的大小

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute(keypoint_indices);

		//在3Dviewer显示提取的特征信息
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
		keypoints.points.resize(keypoint_indices.points.size());
		for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
			keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		//在关键点提取NARF描述子
		std::vector<int> keypoint_indices2;
		keypoint_indices2.resize(keypoint_indices.points.size());
		for (unsigned int i = 0; i < keypoint_indices.size(); ++i)
			keypoint_indices2[i] = keypoint_indices.points[i];					//建立NARF关键点的索引向量，此矢量作为NARF特征计算的输入来使用

		pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);	//创建narf_descriptor对象。并给了此对象输入数据（特征点索引和深度像）
		narf_descriptor.getParameters().support_size = support_size;			//support_size确定计算描述子时考虑的区域大小
		narf_descriptor.getParameters().rotation_invariant = rotation_invariant;    //设置旋转不变的NARF描述子
		pcl::PointCloud<pcl::Narf36> narf_descriptors;							//创建Narf36的点类型输入点云对象并进行实际计算
		narf_descriptor.compute(narf_descriptors);								//计算描述子
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//重设视角
		viewer->resetCamera();
		ui.qvtkWidget->update();
		ui.textBrowser->append("特征点提取成功");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("特征点提取失败：未加载点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
	
}

//区域增长分割龙龙
void PointCloudVision::on_action_segmentation_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始区域增长分割");
		ui.textBrowser->update(); QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("区域增长分割失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();
		return;
	}

	son_dragon.show();
}

//龙龙区域分割
void PointCloudVision::segmentation_calculate_by_choice(int k_choice, double DisThre, int minSize, double normalWeight, int maxite, int maxError)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<PointT>::Ptr cloud_all(new pcl::PointCloud<PointT>);
	PointT start, end;
	int errorCount = 0;
	int m = 0;
	int R, G, B;

	//法向量计算
	pcl::NormalEstimation<PointT, pcl::Normal> normObj;  //创建法向量计算对象
	normObj.setInputCloud(m_currentCloud);                  //设置输入点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr KDtree(new pcl::search::KdTree<pcl::PointXYZ>());
	normObj.setSearchMethod(KDtree); //设置搜索方法
	normObj.setKSearch(30);       //设置K邻域搜索
	pcl::PointCloud<pcl::Normal>::Ptr Pnormals(new pcl::PointCloud<pcl::Normal>); //创建法向量点云
	normObj.compute(*Pnormals); //计算法向量
	normObj.setKSearch(30);       //设置K邻域搜索
	normObj.compute(*Pnormals); //计算法向量

	on_action_reset_triggered();
	cloud_input->points = m_currentCloud->points;
	seg.setOptimizeCoefficients(true);
	if (k_choice == 1)
	{
		seg.setModelType(pcl::SACMODEL_CYLINDER); 		//设置分割模型为圆柱型
	}
	else if (k_choice == 2)
	{
		seg.setModelType(pcl::SACMODEL_SPHERE);
	}
	else if (k_choice == 0)
	{
		seg.setModelType(pcl::SACMODEL_PLANE);
	}

	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxite);
	seg.setDistanceThreshold(DisThre);
	seg.setNormalDistanceWeight(normalWeight);;
	seg.setInputCloud(m_currentCloud);
	seg.setInputNormals(Pnormals);

	int k_num = k_choice == 0 ? maxite : maxError;

	while (errorCount < k_num)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
		seg.setInputCloud(m_currentCloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			errorCount++;
			continue;
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(m_currentCloud);
		extract.setIndices(inliers);
		extract.filter(*cloud_plane);

		// 可视化相关的代码
		if (cloud_plane->points.size() > minSize)
		{
			m++;
			errorCount = 0;
			R = rand() % (256) + 0;
			G = rand() % (256) + 0;
			B = rand() % (256) + 0;
			pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
			for (std::size_t k = 0; k < cloud_plane->size(); k++)
			{
				PointT thispoint;
				thispoint.x = cloud_plane->points[k].x;
				thispoint.y = cloud_plane->points[k].y;
				thispoint.z = cloud_plane->points[k].z;
				cloud_cluster->push_back(thispoint);
			}
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorP(cloud_cluster, R, G, B);

			if (k_choice == 1)
			{
				viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, colorP, "cylinder" + to_string(m));
				//输出圆柱的信息
				string str = "圆柱_" + to_string(m) + "信息：\n";
				//str.append("轴心起点坐标为(").append(to_string(start.data[0])).append(",").append(to_string(start.data[1])).append(",").append(to_string(start.data[2])).append(")\n");
				//str.append("轴心终点坐标为(").append(to_string(end.data[0])).append(",").append(to_string(end.data[1])).append(",").append(to_string(end.data[2])).append(")\n");
				str.append("轴心向量为：(").append(to_string(coefficients->values[3])).append(",").append(to_string(coefficients->values[4])).append(",").append(to_string(coefficients->values[5])).append(")\n");
				str.append("底面半径为：").append(to_string(coefficients->values[6]));
				ui.textBrowser->append(QString::fromStdString(str));
			}
			else if (k_choice == 2)
			{
				viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, colorP, "circle" + to_string(m));

				//输出球面的信息
				string str = "球_" + to_string(m) + "信息：\n";
				str.append("球心坐标为(").append(to_string(coefficients->values[0])).append(",").append(to_string(coefficients->values[1])).append(",").append(to_string(coefficients->values[2])).append(")\n");
				str.append("球半径为：").append(to_string(coefficients->values[3]));
				ui.textBrowser->append(QString::fromStdString(str));
			}
			else if (k_choice == 0)
			{
				viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, colorP, "plane" + to_string(m));
				//输出平面方程
				string s = "平面_" + to_string(m) + "方程为";
				s.append(to_string(coefficients->values[0]));
				s.append("*x");
				if (to_string(coefficients->values[1]).at(0) != '-')
				{
					s.append("+");
				}
				s.append(to_string(coefficients->values[1]));
				s.append("*y");
				if (to_string(coefficients->values[2]).at(0) != '-')
				{
					s.append("+");
				}
				s.append(to_string(coefficients->values[2]));
				s.append("*z");
				if (to_string(coefficients->values[3]).at(0) != '-')
				{
					s.append("+");
				}
				s.append(to_string(coefficients->values[3]));
				s.append("=0");
				ui.textBrowser->append(QString::fromStdString(s));
			}
		}
		else
		{
			errorCount++;
		}
		// 移除cylinder
		extract.setNegative(true);
		extract.filter(*cloud_p);
		*m_currentCloud = *cloud_p;
	}
	ui.textBrowser->append(QString::fromStdString("一共分割出" + to_string(m) + "个面"));
	m_currentCloud->points = cloud_input->points;
	
}

//滤波-张喆-zz
float get_ave_voxel_size(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//构建KD树
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	//计算平均点间距
	float avgdis = 0;
	float n = 0;
	for (int i = 0; i < cloud->size(); i++)
	{
		std::vector<int> pointID;
		std::vector<float> pointDIS;
		if (tree.nearestKSearch(cloud->points[i], 2, pointID, pointDIS) > 0)
		{
			avgdis = avgdis + sqrt(pointDIS[1]);
			n++;
		}
	}
	avgdis = avgdis / n;
	return avgdis / sqrt(3);
}

//张喆-滤波
void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int times, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);             //输入点云
	float ave_dis = get_ave_voxel_size(cloud);
	//std::cout << voxel_size[0] << " " << voxel_size[1] << " " << voxel_size[2];
	sor.setLeafSize(ave_dis * times, ave_dis * times, ave_dis * times); //体素滤波器，单位m
	sor.filter(*cloud_filtered);          //滤波后的点云
}

void PointCloudVision::filter_dialog_function(int times)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	filter(m_currentCloud, times, cloud_filtered);

	viewer->removeAllPointClouds();
	m_currentCloud = cloud_filtered;
	viewer->addPointCloud(m_currentCloud);
	ui.textBrowser->append("过滤结束");
	ui.textBrowser->update(); QApplication::processEvents();
}

//滤波-张喆-zz
void PointCloudVision::on_action_filter_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始过滤");
		ui.textBrowser->update(); QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("过滤失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();
		return;
	}
	filter_dialog.show();
	
}

//区域增长分割;
void PointCloudVision::on_action_grow_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始区域增长分割");
		ui.textBrowser->update(); QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("区域增长分割失败：未加载点云");
		ui.textBrowser->update(); QApplication::processEvents();
		return;
	}

	//设置默认输入参数
	int KN_normal = 30;
	float SmoothnessThreshold = 5, CurvatureThreshold = 200;

	//法线估计
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);//创建一个指向kd树搜索对象的共享指针
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//创建法线估计对象
	normal_estimator.setSearchMethod(tree);//设置搜索方法
	normal_estimator.setInputCloud(m_currentCloud);//设置法线估计对象输入点集
	normal_estimator.setKSearch(KN_normal);// 设置用于法向量估计的k近邻数目
	normal_estimator.compute(*normals);//计算并输出法向量

									   // 区域生长算法的5个参数
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;//创建区域生长分割对象
	reg.setMinClusterSize(50);//设置一个聚类需要的最小点数
	reg.setMaxClusterSize(m_currentCloud->points.size());//设置一个聚类需要的最大点数
	reg.setSearchMethod(tree);//设置搜索方法
	reg.setNumberOfNeighbours(15);//设置搜索的临近点数目
	reg.setInputCloud(m_currentCloud);//设置输入点云
	pcl::IndicesPtr indices(new std::vector <int>);//创建一组索引
	reg.setInputNormals(normals);//设置输入点云的法向量
	reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI);//设置平滑阈值
	reg.setCurvatureThreshold(CurvatureThreshold);//设置曲率阈值
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);//获取聚类的结果，分割结果保存在点云索引的向量中。

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> RegionGrow; //用于储存区域增长分割后的点云
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(m_currentCloud->points[*pit]);
		cloud_cluster->width = m_currentCloud->width;
		cloud_cluster->height = m_currentCloud->height;
		RegionGrow.push_back(cloud_cluster);
		ui.textBrowser->append(QString::fromStdString("plane" + to_string(it - clusters.begin()) + "分割成功"));
		ui.textBrowser->update(); QApplication::processEvents();
	}
	ui.textBrowser->append(QString::fromStdString(to_string(RegionGrow.size()) + "个面分割成功"));
	ui.textBrowser->update(); QApplication::processEvents();

	//可视化
	for (int i = 0; i < RegionGrow.size(); i++)
	{
		//显示分割得到的各片点云 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorP(RegionGrow[i], 255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2);
		viewer->addPointCloud<pcl::PointXYZ>(RegionGrow[i], colorP, " clous_segment_" + to_string(i));
	}

}
//PCA-ICP
void PointCloudVision::on_action_icp_triggered()
{
	ui.textBrowser->append("功能未定义");
	ui.textBrowser->update();

}

//SCALE-ICP
void PointCloudVision::on_action_action_scale_icp_triggered()
{
	ui.textBrowser->append("功能未定义");
	ui.textBrowser->update();

}

//体素滤波
void PointCloudVision::on_action_3_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始体素滤波");
		ui.textBrowser->update();
		QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();
		pcl::VoxelGrid<pcl::PointXYZ> vg;		//创建滤波器对象
		vg.setInputCloud(m_currentCloud);				//设置待滤波点云
		vg.setLeafSize(1.15f, 1.15f, 1.15f);	//设置体素大小
		vg.filter(*m_currentCloud);			//执行滤波，保存滤波结果于cloud_filtered
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("体素滤波成功");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("体素滤波失败：未加载点云");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//直通滤波;
void PointCloudVision::on_action_4_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始直通滤波");
		ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::PassThrough<pcl::PointXYZ> pass;	//创建直通滤波器对象
		pass.setInputCloud(m_currentCloud);		        //设置输入的点云
		pass.setFilterFieldName("z");           //设置过滤时所需要点云类型为Z字段
		pass.setFilterLimits(-0.1, 10);         //设置在过滤字段的范围
		pass.setFilterLimitsNegative(true);     //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
		pass.filter(*m_currentCloud);		    //执行滤波
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("直通滤波成功");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("直通滤波失败：未加载点云文件");
		ui.textBrowser->update();QApplication::processEvents();

	}
}

//统计滤波;
void PointCloudVision::on_action_5_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始统计滤波");
		ui.textBrowser->update();QApplication::processEvents();
		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建统计滤波器对象 
		sor.setInputCloud(m_currentCloud);         			         //设置输入的点云
		sor.setMeanK(50);                 					 //设置KNN的k值
		sor.setStddevMulThresh(1.0);      				     //设置标准偏差乘数为1.0
		sor.filter(*m_currentCloud);          			     //执行滤波
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("统计滤波成功");
		 ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("统计滤波失败，未加载点云文件");
		ui.textBrowser->update();QApplication::processEvents();

	}
}

//均匀采样滤波;
void PointCloudVision::on_action_6_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始均匀采样滤波");
		ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();


		pcl::UniformSampling<pcl::PointXYZ> unisam;
		unisam.setInputCloud(m_currentCloud);
		unisam.setRadiusSearch(0.01f);
		unisam.filter(*m_currentCloud);
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("均匀采样滤波成功");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("均匀采样滤波失败，未加载点云文件");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//半径滤波
void PointCloudVision::on_action_7_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("开始半径滤波");
		ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(m_currentCloud);
		outrem.setRadiusSearch(0.001);                     //设置半径为0.1的范围内找临近点
		outrem.setMinNeighborsInRadius(2);               //设置查询点的邻域点集数小于2的删除
		outrem.filter(*m_currentCloud);                  //执行滤波
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("半径滤波成功");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("均匀采样滤波失败，未加载点云文件");
		ui.textBrowser->update();QApplication::processEvents();

	}
}

double getMinValue(PointT p1, PointT p2)
{
	double min = 0;

	if (p1.x - p2.x > p1.y - p2.y)
	{
		min = p1.y - p2.y;
	}
	else
	{
		min = p1.x - p2.x;
	}

	if (min > p1.z - p2.z)
	{
		min = p1.z - p2.z;
	}

	return min;
}


double getMaxValue(PointT p1, PointT p2)
{
	double max = 0;

	if (p1.x - p2.x > p1.y - p2.y)
	{
		max = p1.x - p2.x;

	}
	else
	{
		max = p1.y - p2.y;
	}

	if (max < p1.z - p2.z)
	{
		max = p1.z - p2.z;
	}

	return max;
}

void setCoordinate1(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud)
{
	PointT p_min, p_max;
	//viewer->addPointCloud(m_currentCloud);
	double point[3];
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
	getCenterPoint(m_currentCloud, point);
	double scale = getMaxValue(p_max, p_min);
	double maxLen = getMaxValue(p_max, p_min);

	/*for (int i = 0; i < m_currentCloud->points.size(); i++)
	{
	m_currentCloud->points[i].x = m_currentCloud->points[i].x - point[0];
	m_currentCloud->points[i].y = m_currentCloud->points[i].y - point[1];
	m_currentCloud->points[i].z = m_currentCloud->points[i].x - point[0];
	}*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//定义平移矩阵，并初始化为单位阵
	translation(0, 3) = -cloudCentroid[0];
	translation(1, 3) = -cloudCentroid[1];
	translation(2, 3) = -cloudCentroid[2];
	pcl::transformPointCloud(*m_currentCloud, *cloud_transformed, translation);

	viewer->removeAllPointClouds();
	pcl::copyPointCloud(*cloud_transformed, *m_currentCloud);
	//m_currentCloud = cloud_transformed;
	viewer->addPointCloud(m_currentCloud);
	viewer->removeAllCoordinateSystems();
	viewer->addCoordinateSystem(scale*0.5, cloudCentroid[0], cloudCentroid[1], cloudCentroid[2], 0);
}

void setCoordinate(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud)
{
	PointT p_min, p_max;
	//viewer->addPointCloud(m_currentCloud);
	double point[3];
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
	getCenterPoint(m_currentCloud, point);
	double scale = getMaxValue(p_max, p_min);
	double maxLen = getMaxValue(p_max, p_min);
	
	/*for (int i = 0; i < m_currentCloud->points.size(); i++)
	{
		m_currentCloud->points[i].x = m_currentCloud->points[i].x - point[0];
		m_currentCloud->points[i].y = m_currentCloud->points[i].y - point[1];
		m_currentCloud->points[i].z = m_currentCloud->points[i].x - point[0];
	}*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//计算点云质心
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//定义平移矩阵，并初始化为单位阵
	translation(0, 3) = -point[0];
	translation(1, 3) = -point[1];
	translation(2, 3) = -point[2];
	pcl::transformPointCloud(*m_currentCloud, *cloud_transformed, translation);

	viewer->removeAllPointClouds();
	pcl::copyPointCloud(*cloud_transformed, *m_currentCloud);
	//m_currentCloud = cloud_transformed;
	viewer->addPointCloud(m_currentCloud);
	viewer->removeAllCoordinateSystems();
	viewer->addCoordinateSystem(scale*0.5, 0, 0, 0, 0);
}

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

void exercise() 
{
	
}

//求单个点的法向量
vector<double> computeNormal_(vector<PointT> points) {

	const int num = points.size();//临近点的数量
	Eigen::MatrixXd errorMatrix(3, 3);
	Eigen::EigenSolver<Eigen::MatrixXd> es(errorMatrix);
	Eigen::MatrixXcd evalsReal;//记录下特征值的实数部分
	Eigen::MatrixXd eigenVector;//记录下特征向量
	int valueMin;//记录下最小特征值的索引
	vector<double > res;
	//质心点坐标
	float x_ = 0;
	float y_ = 0;
	float z_ = 0;


	//计算质心点坐标
	float temp = 0;

	for (int i = 0; i < num; i++) {

		temp += points[i].x;
	}
	x_ = temp / num;

	temp = 0;

	for (int i = 0; i < num; i++) {

		temp += points[i].y;
	}
	y_ = temp / num;

	temp = 0;

	for (int i = 0; i < num; i++) {

		temp += points[i].z;
	}
	z_ = temp / num;


	errorMatrix(0, 0) = 0;
	errorMatrix(0, 1) = 0;
	errorMatrix(0, 2) = 0;
	errorMatrix(1, 0) = 0;
	errorMatrix(1, 1) = 0;
	errorMatrix(1, 2) = 0;
	errorMatrix(2, 0) = 0;
	errorMatrix(2, 1) = 0;
	errorMatrix(2, 2) = 0;

	//生成坐标误差矩阵
	for (int j = 0; j < num; j++) {

		errorMatrix(0, 0) += (points[j].x - x_) * (points[j].x - x_);
		errorMatrix(0, 1) += (points[j].x - x_) * (points[j].y - y_);
		errorMatrix(0, 2) += (points[j].x - x_) * (points[j].z - z_);
		errorMatrix(1, 0) += (points[j].x - x_) * (points[j].y - y_);
		errorMatrix(1, 1) += (points[j].y - y_) * (points[j].y - y_);
		errorMatrix(1, 2) += (points[j].z - z_) * (points[j].y - y_);
		errorMatrix(2, 0) += (points[j].x - x_) * (points[j].z - z_);
		errorMatrix(2, 1) += (points[j].y - y_) * (points[j].z - z_);
		errorMatrix(2, 2) += (points[j].z - z_) * (points[j].z - x_);
	}

	double min = INT_MAX * 1.0;//记录下最小特征值对应的索引值
	evalsReal = es.eigenvalues().real();

	eigenVector = es.eigenvectors().real();
	
	for (int i = 0; i < 3; i++) {

		if (evalsReal(i, 0).real() < min) {

			valueMin = i;
			min = evalsReal(i, 0).real();
		}
	}
	for (int i = 0; i < 3; i++) {

		res.push_back(eigenVector(i, valueMin));
	}

	return  res;
}

void PointCloudVision::drawArrow(PointT start, PointT end, string id, vector<int> RGB) {

	vector<float> vec = { end.x - start.x, end.y - start.y, end.z - start.z };
	PointT *l = new PointT();
	PointT *r = new PointT();

	string s = "向量" + id  + "坐标为(" + to_string(vec[0]) + "," + to_string(vec[1]) + " ," + to_string(vec[2]) + ")";
	ui.textBrowser->append(QString::fromStdString(s));
	l->x = start.x + vec[0] * 4 / 3;
	l->y = start.y + vec[0] * 2 / 3;
	l->z = start.z + vec[0] * 2 / 3;

	r->x = start.x;
	r->y = start.y + vec[0] * 2 / 3;
	r->z = start.z + vec[0] * 2 / 3;


	viewer->addLine<PointT>(start, end);
	viewer->addLine<PointT>(end, *l);
	viewer->addLine<PointT>(end, *r);
	ui.textBrowser->append(QString::fromStdString("已画出" + id));
	return;
}