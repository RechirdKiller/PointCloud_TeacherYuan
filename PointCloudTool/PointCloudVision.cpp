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

	//��ʼ��
	init();


}

//��ȡ������ƽ�������������̾���
double getMinValue(PointT p1, PointT p2);

//��ȡ������ƽ����������������
double getMaxValue(PointT p1, PointT p2);

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

//��ʼ��
void PointCloudVision::init()
{
	//���Ƴ�ʼ��
	m_currentCloud.reset(new PointCloudT);

	//���ӻ������ʼ��
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	
	//����VTK���ӻ�����ָ��
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//���ô��ڽ��������ڿɽ��ܼ��̵��¼�
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());

	//���������
	viewer->addCoordinateSystem(1, 0);

	//�ۺ���
	connect(&heightRampDlg, SIGNAL(setHeightRamp(int, double)), this, SLOT(setHeightRamp(int, double)));
}

//�򿪵���
void PointCloudVision::on_action_open_triggered()
{


	//��ȡ����·��
	QString path = QFileDialog::getOpenFileName(this, "ѡ������ļ�", ".//", "�����ļ�(*.txt *.pcd *.ply);;�����ļ�(*.*)");

	//��ȡ��������
	QFile file(path);

	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		//��յ���
		m_currentCloud->clear();
		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		string Path = path.toStdString();
		string suffixStr = Path.substr(Path.find_last_of('.') + 1);//��ȡ�ļ���׺
		if (suffixStr == "pcd") {
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				QMessageBox::warning(this, "����", "�����ļ���ʽ����");
			}
		}
		else if (suffixStr == "ply") {
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				QMessageBox::warning(this, "����", "�����ļ���ʽ����");
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
					QMessageBox::warning(this, "����", "�����ļ���ʽ����");
					break;
				}


				//���Ƹ�ֵ
				PointT point;
				point.x = strList.at(0).toDouble();
				point.y = strList.at(1).toDouble();
				point.z = strList.at(2).toDouble();

				m_currentCloud->push_back(point);
			}
		}
		
		//��ӵ�����
		viewer->addPointCloud(m_currentCloud);

		pcl::getMinMax3D(*m_currentCloud, p_min, p_max);

		double scale = getMinValue(p_max, p_min);
		maxLen = getMaxValue(p_max, p_min);
		viewer->addCoordinateSystem(scale*0.3, 0);

		//�����ӽ�
		viewer->resetCamera();

		//ˢ�´���
		ui.qvtkWidget->update();
	}
	file.close();


}

//�����ӽ�
void PointCloudVision::on_action_reset_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}

//����ͼ
void PointCloudVision::on_action_up_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0, 0, 1, 0);
		ui.qvtkWidget->update();
	}
}

//ǰ��ͼ
void PointCloudVision::on_action_front_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_min.y - 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), 0, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//����ͼ
void PointCloudVision::on_action_left_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//����ͼ
void PointCloudVision::on_action_back_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_max.y + 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), 0, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//����ͼ
void PointCloudVision::on_action_right_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_max.x + 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui.qvtkWidget->update();
	}
}

//����ͼ
void PointCloudVision::on_action_bottom_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_min.z - 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0, 0, 1, 0);
		ui.qvtkWidget->update();
	}
}

//ǰ���
void PointCloudVision::on_action_frontIso_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, p_min.y - 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 1, 1, 0);
		ui.qvtkWidget->update();
	}
}

//�����
void PointCloudVision::on_action_backIso_triggered()
{
	viewer->setCameraPosition(p_max.x + 2 * maxLen, p_max.y + 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), -1, -1, 0);
	ui.qvtkWidget->update();
}

//���õ�����ɫ
void PointCloudVision::on_action_setColor_triggered()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "���õ�����ɫ", QColorDialog::ShowAlphaChannel);

	viewer->removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> singelColor(m_currentCloud, color.red(), color.green(), color.blue());
	viewer->addPointCloud(m_currentCloud, singelColor, "myCloud", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, color.alpha()*1.0 / 255, "myCloud");

	ui.qvtkWidget->update();

}

//���ø߶���Ⱦ
void PointCloudVision::on_action_heightRamp_triggered()
{
	heightRampDlg.show();
}

//���и߶���Ⱦ
void PointCloudVision::setHeightRamp(int dir, double height)
{
	//��յ���
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

		pcl::PassThrough<PointT> pass;			//ֱͨ�˲�������
		pass.setInputCloud(m_currentCloud);		//�������
		pass.setFilterFieldName(field);			//���ù����ֶ�
		pass.setFilterLimits(i, i + height);	//���ù��˷�Χ
		pass.setFilterLimitsNegative(false);	//���ñ����ֶ�
		pass.filter(*cloudTemp);				//ִ���˲�

		i += height;

		m_heightCloudList.append(cloudTemp);
	}

	//�ֶ���Ⱦ
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
	//���߹��ƶ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//�洢���Ƶķ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//����kd��ָ��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(m_currentCloud);
	n.setInputCloud(m_currentCloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	//���Ʒ��ߴ洢������
	n.compute(*normals);//Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
	//�����ֶ�
	pcl::concatenateFields(*m_currentCloud, *normals, *cloud_width_normals);

	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//���ƹ���������
	tree2->setInputCloud(cloud_width_normals);

	//�������ǻ�����
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//�洢�������ǻ�������ģ��
	pcl::PolygonMesh triangles;//�������ӵ�֮��������룬���������������߳���
	gp3.setSearchRadius(200.0f);
	//���ø��ֲ���ֵ
	gp3.setMu(2.5f);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI_4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);

	//���������������������
	gp3.setInputCloud(cloud_width_normals);
	gp3.setSearchMethod(tree2);

	//ִ���ع������������triangles��
	gp3.reconstruct(triangles);
	viewer->addPolygonMesh(triangles, "my");
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);

	double scale = getMinValue(p_max, p_min);
	maxLen = getMaxValue(p_max, p_min);
	viewer->addCoordinateSystem(scale*0.3, 0);

	//�����ӽ�
	viewer->resetCamera();

	//ˢ�´���
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

	//��һ�������е�.pcd�ļ�  �������û��ָ���ͻ��Զ�����
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
	//�ӵ����н����������ͼ
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

	//��3D viewer���������
	//pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer->addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	viewer->initCameraParameters();
	setViewerPose(*viewer, range_image.getTransformationToWorldSystem());
	//��ȡNARF����
	pcl::RangeImageBorderExtractor range_image_border_extractor;    //�������ͼ��Ե��ȡ��
	pcl::NarfKeypoint narf_keypoint_detector;                       //narf_keypoint_detectorΪ���ƶ���

	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;    //���������ȡ�Ĵ�С

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);

	//��3Dviewer��ʾ��ȡ��������Ϣ
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i<keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	//�ڹؼ�����ȡNARF������
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize(keypoint_indices.points.size());
	for (unsigned int i = 0; i<keypoint_indices.size(); ++i)
		keypoint_indices2[i] = keypoint_indices.points[i];					//����NARF�ؼ����������������ʸ����ΪNARF���������������ʹ��

	pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);	//����narf_descriptor���󡣲����˴˶����������ݣ������������������
	narf_descriptor.getParameters().support_size = support_size;			//support_sizeȷ������������ʱ���ǵ������С
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;    //������ת�����NARF������
	pcl::PointCloud<pcl::Narf36> narf_descriptors;							//����Narf36�ĵ�����������ƶ��󲢽���ʵ�ʼ���
	narf_descriptor.compute(narf_descriptors);								//����������
	double scale = getMinValue(p_max, p_min);
	maxLen = getMaxValue(p_max, p_min);
	viewer->addCoordinateSystem(scale*0.3, 0);
	//�����ӽ�
	viewer->resetCamera();
	ui.qvtkWidget->update();
	
}
void PointCloudVision::on_action_grow_triggered()
{
	viewer->removeAllPointClouds();
	viewer->removeAllCoordinateSystems();
	viewer->removeAllShapes();
	//���������ķ�ʽ����˵�ǽṹ
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//����
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(m_currentCloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
	//ֱͨ�˲���Z���0��1��֮��
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(m_currentCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
	//�������<�㣬����>
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;			//���Ƚ���reg�Ĵ���(���������Ķ���
	reg.setMinClusterSize(50);									//��С�ľ���ĵ���(С���������ƽ�汻���Բ��ƣ�
	reg.setMaxClusterSize(1000000);								//����(һ��������ã�
	reg.setSearchMethod(tree);									//������ʽ(���õ�Ĭ����K��d������
	reg.setNumberOfNeighbours(30);								//���������������ĸ�������Χ���ٸ����������һ��ƽ��(�����ݴ��ʣ����ô�ʱ����бҲ�ɽ��ܣ�����Сʱ��⵽��ƽ����С��
	reg.setInputCloud(m_currentCloud);							//�����
	reg.setInputNormals(normals);								//����ķ���
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);				//����ƽ����(�������������ڶ��н��ڿɵ����ǹ���ģ�
	reg.setCurvatureThreshold(1.0);								//�������ʵ���ֵ
																//���Ҳ��һ����������ֵ����������˱ȵ�ǰ����ĵ��ƽ���ķ��߽Ƕȣ������Ƿ��м���̽����ȥ�ı�Ҫ��
																//��Ҳ���Ǽ���ÿ���㶼��ƽ�������ģ���ônormal�ļнǶ���С������ʱ�䳤��ƫ�Ƶľʹ��ˣ��������������������õģ�

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
	//�����ӽ�
	viewer->resetCamera();
	//ˢ�´���
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