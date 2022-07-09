#include "PointCloudVision.h"
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QColorDialog>
#include <vtkRenderWindow.h>
using namespace std;
#pragma execution_character_set("utf-8")
PointCloudVision::PointCloudVision(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//初始化
	init();


}

//获取两个点平行于坐标轴的最短距离
double getMinValue(PointT p1, PointT p2);

//获取两个点平行于坐标轴的最长距离
double getMaxValue(PointT p1, PointT p2);

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

//初始化
void PointCloudVision::init()
{
	//点云初始化
	m_currentCloud.reset(new PointCloudT);

	//可视化对象初始化
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	
	//设置VTK可视化窗口指针
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//设置窗口交互，窗口可接受键盘等事件
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());

	//添加坐标轴
	viewer->addCoordinateSystem(1, 0);

	//槽函数
	connect(&heightRampDlg, SIGNAL(setHeightRamp(int, double)), this, SLOT(setHeightRamp(int, double)));
}

//打开点云
void PointCloudVision::on_action_open_triggered()
{


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
		string suffixStr = Path.substr(Path.find_last_of('.') + 1);//获取文件后缀
		if (suffixStr == "pcd") {
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				QMessageBox::warning(this, "警告", "点云文件格式错误！");
			}
		}
		else if (suffixStr == "ply") {
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				QMessageBox::warning(this, "警告", "点云文件格式错误！");
			}
		}
		
		else if(suffixStr == "txt") {
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
					QMessageBox::warning(this, "警告", "点云文件格式错误！");
					break;
				}


				//点云赋值
				PointT point;
				point.x = strList.at(0).toDouble();
				point.y = strList.at(1).toDouble();
				point.z = strList.at(2).toDouble();

				m_currentCloud->push_back(point);
			}
		}
		
		//添加到窗口
		viewer->addPointCloud(m_currentCloud);

		pcl::getMinMax3D(*m_currentCloud, p_min, p_max);

		double scale = getMinValue(p_max, p_min);
		maxLen = getMaxValue(p_max, p_min);
		viewer->addCoordinateSystem(scale*0.3, 0);

		//重设视角
		viewer->resetCamera();

		//刷新窗口
		ui.qvtkWidget->update();
	}
	file.close();


}

//重设视角
void PointCloudVision::on_action_reset_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}

//俯视图
void PointCloudVision::on_action_up_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0, 0, 1, 0);
		ui.qvtkWidget->update();
	}
}

//前视图
void PointCloudVision::on_action_front_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_min.y - 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), 0, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//左视图
void PointCloudVision::on_action_left_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//后视图
void PointCloudVision::on_action_back_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_max.y + 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), 0, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//右视图
void PointCloudVision::on_action_right_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_max.x + 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//底视图
void PointCloudVision::on_action_bottom_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_min.z - 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0, 0, 1, 0);
		ui.qvtkWidget->update();
	}
}

//前轴测
void PointCloudVision::on_action_frontIso_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, p_min.y - 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 1, 1, 0);
		ui.qvtkWidget->update();
	}
}

//后轴测
void PointCloudVision::on_action_backIso_triggered()
{
	viewer->setCameraPosition(p_max.x + 2 * maxLen, p_max.y + 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), -1, -1, 0);
	ui.qvtkWidget->update();
}

//设置点云颜色
void PointCloudVision::on_action_setColor_triggered()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "设置点云颜色", QColorDialog::ShowAlphaChannel);

	viewer->removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> singelColor(m_currentCloud, color.red(), color.green(), color.blue());
	viewer->addPointCloud(m_currentCloud, singelColor, "myCloud", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, color.alpha()*1.0 / 255, "myCloud");

	ui.qvtkWidget->update();

}

//设置高度渲染
void PointCloudVision::on_action_heightRamp_triggered()
{
	heightRampDlg.show();
}

//进行高度渲染
void PointCloudVision::setHeightRamp(int dir, double height)
{
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

}
void PointCloudVision::on_action_triangle_triggered()
{
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
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);

	double scale = getMinValue(p_max, p_min);
	maxLen = getMaxValue(p_max, p_min);
	viewer->addCoordinateSystem(scale*0.3, 0);

	//重设视角
	viewer->resetCamera();

	//刷新窗口
	ui.qvtkWidget->update();
	
}
void PointCloudVision::on_action_feature_triggered()
{
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
	for (int i = 0; i <M; i++)
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
	for (size_t i = 0; i<keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	//在关键点提取NARF描述子
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize(keypoint_indices.points.size());
	for (unsigned int i = 0; i<keypoint_indices.size(); ++i)
		keypoint_indices2[i] = keypoint_indices.points[i];					//建立NARF关键点的索引向量，此矢量作为NARF特征计算的输入来使用

	pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);	//创建narf_descriptor对象。并给了此对象输入数据（特征点索引和深度像）
	narf_descriptor.getParameters().support_size = support_size;			//support_size确定计算描述子时考虑的区域大小
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;    //设置旋转不变的NARF描述子
	pcl::PointCloud<pcl::Narf36> narf_descriptors;							//创建Narf36的点类型输入点云对象并进行实际计算
	narf_descriptor.compute(narf_descriptors);								//计算描述子
	double scale = getMinValue(p_max, p_min);
	maxLen = getMaxValue(p_max, p_min);
	viewer->addCoordinateSystem(scale*0.3, 0);
	//重设视角
	viewer->resetCamera();
	ui.qvtkWidget->update();
	
}
void PointCloudVision::on_action_grow_triggered()
{
	viewer->removeAllPointClouds();
	viewer->removeAllCoordinateSystems();
	viewer->removeAllShapes();
	//设置搜索的方式或者说是结构
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//求法线
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(m_currentCloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
	//直通滤波在Z轴的0到1米之间
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(m_currentCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
	//聚类对象<点，法线>
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;			//首先建立reg寄存器(区域增长的对象）
	reg.setMinClusterSize(50);									//最小的聚类的点数(小于这参数的平面被忽略不计）
	reg.setMaxClusterSize(1000000);								//最大的(一般随便设置）
	reg.setSearchMethod(tree);									//搜索方式(采用的默认是K—d树法）
	reg.setNumberOfNeighbours(30);								//设置搜索的邻域点的个数，周围多少个点决定这是一个平面(决定容错率，设置大时有倾斜也可接受，设置小时检测到的平面会很小）
	reg.setInputCloud(m_currentCloud);							//输入点
	reg.setInputNormals(normals);								//输入的法线
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);				//设置平滑度(设置两个法线在多大夹角内可当做是共面的）
	reg.setCurvatureThreshold(1.0);								//设置曲率的阈值
																//最后也是一个弯曲的阈值，这个决定了比当前考察的点和平均的法线角度，决定是否还有继续探索下去的必要。
																//（也就是假设每个点都是平稳弯曲的，那么normal的夹角都很小，但是时间长了偏移的就大了，这个参数就是限制这个用的）

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	viewer->addPointCloud(colored_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	int M = colored_cloud->points.size();
	for (int i = 0; i <M; i++)
	{
		pcl::PointXYZ p;
		p.x = colored_cloud->points[i].x;
		p.y = colored_cloud->points[i].y;
		p.z = colored_cloud->points[i].z;
		cloud->points.push_back(p);
	}
	cloud->width = 1;
	cloud->height = M;
	pcl::getMinMax3D(*cloud, p_min, p_max);
	double scale = getMinValue(p_max, p_min);
	maxLen = getMaxValue(p_max, p_min);
	viewer->addCoordinateSystem(scale*0.3, 0);
	//重设视角
	viewer->resetCamera();
	//刷新窗口
	ui.qvtkWidget->update();

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

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}