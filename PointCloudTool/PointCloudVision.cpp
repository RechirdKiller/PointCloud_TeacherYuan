#include "PointCloudVision.h"
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QColorDialog>
#include <QToolButton>
#include <vtkRenderWindow.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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
#include"octree_search_my.h"
#include <Eigen/Core>

using namespace std;
#pragma execution_character_set("utf-8")

QString Last_FileName=nullptr;
double dist_scale = 1;

PointCloudVision::PointCloudVision(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//��ʼ��
	init();
	//dist_scale = 1;

}

//��ȡ������ƽ�������������̾���
double getMinValue(PointT p1, PointT p2);

//��ȡ������ƽ����������������
double getMaxValue(PointT p1, PointT p2);

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
//����������е�
double* getCenterPoint(PointT p1, PointT p2);

//���е�Ϊ���ģ�����ά�ȵ���С�����0.3��Ϊ������ı�̶ȣ�����᳤��Ϊ��С����������;
void setCoordinate(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud);

//��ʾ������xyz��δ��ɣ�
void addOrientationMarkerWidgetAxesToview(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, vtkRenderWindowInteractor* interactor, double x, double y, double x_wide, double y_wide)
{
	/*pcl::PointXYZ a, b, z;
	a.x = 0;
	a.y = 0;
	a.z = 0;
	b.x = 5;
	b.y = 8;
	b.z = 10;
	z.x = 4;
	z.y = 3;
	z.z = 20;
	viewer->addArrow<pcl::PointXYZ>(b, z, 255, 0, 0, "arrow");  //����ͷ

	viewer->addText3D("X", b, 100.0, 255, 255, 255, "XLable", 1);
	viewer->addText3D("Y", z, 100.0, 255, 255, 255, "YLable", 1);*/
}

//��������ԭ��
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
	viewer->setBackgroundColor((double)113/255, (double)110/255, (double)119/255);

	
	//���������
	viewer->addCoordinateSystem(1, 0);

	//�ۺ���
	//�߶���Ⱦ
	connect(&heightRampDlg, SIGNAL(setHeightRamp(int, double)), this, SLOT(setHeightRamp(int, double)));
	//�˲������ؽ�������
	connect(&octreeDialog, SIGNAL(octree_vsearch(double, double, double, double, int, int, int)), this, SLOT(octree_vsearch(double, double, double, double, int, int, int)));
	//octree

	//�����˲�������;
	toolButton = new QToolButton(this);
	//ֻ��ʾͼƬ;
	toolButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
	//���°�ť��������;
	toolButton->setPopupMode(QToolButton::InstantPopup);
	toolButton->setMenu(ui.menu_5);
	toolButton->setToolTip("�˲�");
	toolButton->setIcon(QIcon(":/PointCloudVision/image/filtering.png"));
	ui.mainToolBar->addWidget(toolButton);

	//������׼������;
	toolButton2 = new QToolButton(this);
	//ֻ��ʾͼƬ;
	toolButton2->setToolButtonStyle(Qt::ToolButtonIconOnly);
	//���°�ť��������;
	toolButton2->setPopupMode(QToolButton::InstantPopup);
	toolButton2->setMenu(ui.menu_6);
	toolButton2->setToolTip("������׼");
	toolButton2->setIcon(QIcon(":/PointCloudVision/image/registration.png"));
	ui.mainToolBar->addWidget(toolButton2);
	//״̬��Ϣ��������
	ui.label->setStyleSheet("background-color:rgb(240,240,240);color:rgb(255,0,0);");
	//��ʾ����
	//ui.textBrowser->setStyleSheet("background-color:rgb(230,231,232);color:rgb(255,0,0);");
	//�߿���ɫ

}


//�򿪵���
void PointCloudVision::on_action_open_triggered()
{
	//����������;
	ui.textBrowser->append("ѡ������ļ�...");
	ui.textBrowser->update();
	
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
		Last_FileName = path;
		string suffixStr = Path.substr(Path.find_last_of('.') + 1);//��ȡ�ļ���׺
		if (suffixStr == "pcd") {
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				//��������
				ui.textBrowser->append("���棺�����ļ���ʽ����");
		  ui.textBrowser->update();
		  QApplication::processEvents();

				QMessageBox::warning(this, "����", "�����ļ���ʽ����");
			}
			else {
				//���������򿪳ɹ�
				QString outputString = "�ɹ�����";
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
				ui.statusBar->addPermanentWidget(statusLabel); //��ʵ������Ϣ
			}
		}
		else if (suffixStr == "ply") {
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(Path, *m_currentCloud) == -1) {
				//��������
				ui.textBrowser->append("���棺�����ļ���ʽ����");
		  ui.textBrowser->update();QApplication::processEvents();

				QMessageBox::warning(this, "����", "�����ļ���ʽ����");
			}
			else {
				//���������򿪳ɹ�
				QString outputString = "�ɹ�����";
				outputString += path;
				ui.textBrowser->append(outputString);
		  ui.textBrowser->update();QApplication::processEvents();

				if (!statusLabel == NULL) {
					ui.statusBar->removeWidget(statusLabel);
				}
				statusLabel = new QLabel(path, this);
				ui.statusBar->addPermanentWidget(statusLabel); //��ʵ������Ϣ
			}
		}
		
		else if(suffixStr == "txt") {
			int flag = 0;//1��ʾ�쳣�˳�;
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
					//��������
					ui.textBrowser->append("���棺�����ļ���ʽ����");
			  ui.textBrowser->update();QApplication::processEvents();

					QMessageBox::warning(this, "����", "�����ļ���ʽ����");
					flag = 1;
					break;
				}


				//���Ƹ�ֵ
				PointT point;
				point.x = strList.at(0).toDouble();
				point.y = strList.at(1).toDouble();
				point.z = strList.at(2).toDouble();

				m_currentCloud->push_back(point);
			}
			if (flag == 0) {
				//���������򿪳ɹ�
				QString outputString = "�ɹ�����";
				outputString += path;
				ui.textBrowser->append(outputString);
				ui.textBrowser->update();QApplication::processEvents();

				if (!statusLabel == NULL) {
					ui.statusBar->removeWidget(statusLabel);
				}
				statusLabel = new QLabel(path, this);
				ui.statusBar->addPermanentWidget(statusLabel); //��ʵ������Ϣ
			}
		}
		
		//��ӵ�����
		viewer->addPointCloud(m_currentCloud);
		//viewer->removeAllCoordinateSystems();
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//�����ӽ�
		dist_scale = 1.0;
		viewer->resetCamera();

		//ˢ�´���
		ui.qvtkWidget->update();
	}
	else {
		ui.textBrowser->append("ȡ��ѡ��");
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

//�������
void PointCloudVision::on_action_preserve_triggered()
{
		QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply *.txt)"));
		//outputInDebug(filename.toStdString());
		//PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

		if (filename.isEmpty()) {
			ui.textBrowser->append("�������δ����·��");
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
			ui.textBrowser->append("д�����");
	  ui.textBrowser->update();QApplication::processEvents();

			return;
		}
}

//�����ӽ�
void PointCloudVision::on_action_reset_triggered()
{
	ui.textBrowser->append("�����ӽ�");
	ui.textBrowser->update();

	if (!m_currentCloud->empty())
	{
		viewer->resetCamera();
		ui.qvtkWidget->update();
		ui.textBrowser->append("����ɹ�");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("����ʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//��ͼ
void PointCloudVision::on_action_screenshot_triggered()
{
	if (!m_currentCloud->empty()) {
		//��ɽ�ͼ����
		QScreen *screen = QGuiApplication::primaryScreen();
		QPixmap pixmap = screen->grabWindow(ui.qvtkWidget->winId());

		//�û�ѡȡ�ļ���
		QString filename = QFileDialog::getSaveFileName(this, tr("Save the image"), "", tr("Image (*.jpg *.png)"));

		if (filename.isEmpty()) {
			ui.textBrowser->append("�������δ����·��");
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
			ui.textBrowser->append("д�����");
			ui.textBrowser->update(); QApplication::processEvents();

			return;
		}
		else {

			ui.textBrowser->append("д��ɹ����ļ�λ��Ϊ��" + filename);
			ui.textBrowser->update(); QApplication::processEvents();

		}
		ui.textBrowser->append("��ͼ�ѱ���");
		ui.textBrowser->update(); QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("��ͼʧ�ܣ�δ���ص���");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}



//�������Ŵ�
void PointCloudVision::on_action_magnify_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.qvtkWidget->update();
		/*viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
		ui.qvtkWidget->update();
		ui.textBrowser->append("���ô�С�ɹ�");
		*/
		if (dist_scale < 3) {
			dist_scale += 0.2;
		}else if(dist_scale>=3 && dist_scale<5){
			dist_scale += 0.5;
		}else{
			dist_scale = 5;
			ui.textBrowser->append("�ѵ����Ŵ���");
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
		ui.textBrowser->append("���ô�С�ɹ�");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("���ô�Сʧ�ܣ�δ���ص���");
		ui.textBrowser->update();QApplication::processEvents();
	}
	
}


//��������С
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
			ui.textBrowser->append("�ѵ���С��С����");
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
		//������ӵ���
		viewer->removeAllPointClouds();
		viewer->addPointCloud(m_currentCloud);
		ui.textBrowser->append("���ô�С�ɹ�");
		ui.textBrowser->update(); 
		QApplication::processEvents();
		ui.qvtkWidget->update();

	} else {
		ui.textBrowser->append("���ô�Сʧ�ܣ�δ���ص���");
		ui.textBrowser->update(); QApplication::processEvents();

	}
}

//���ݴ����index�ж��Ƿ������������ƶ�
void PointCloudVision::changeLocationOfObject(int index)
{
	if (!m_currentCloud->empty()) {
		viewer->removeAllPointClouds();
		PointT p_min, p_max;
		double point[3];
		pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
		//�����������ϵ�Ĺ�ģ
		double scale = getMaxValue(p_max, p_min);
		outputInDebug(to_string(scale));

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		// ������y���ϵ�ƽ��scale
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

		//��ʼ�任
		pcl::transformPointCloud(*m_currentCloud, *cloud_transformed, transform_2);
		//���¸�ֵ
		m_currentCloud = cloud_transformed;
		pcl::copyPointCloud(*cloud_transformed, *m_currentCloud);
		//������ӵ���
		viewer->removeAllPointClouds();
		viewer->addPointCloud(m_currentCloud);
		//���¼���������
		Eigen::Vector4f cloudCentroid;
		pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//�����������
		viewer->removeAllCoordinateSystems();
		viewer->addCoordinateSystem(scale*0.5, cloudCentroid[0], cloudCentroid[1], cloudCentroid[2], 0);
		ui.qvtkWidget->update();
	}
	else {
		ui.textBrowser->append("���ô�Сʧ�ܣ�δ���ص���");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}

//����
void PointCloudVision::on_action_topMove_triggered()
{
	changeLocationOfObject(0);
}

//����
void PointCloudVision::on_action_bottomMove_triggered()
{
	changeLocationOfObject(1);
}

//����
void PointCloudVision::on_action_leftMove_triggered()
{
	changeLocationOfObject(3);
}

//����
void PointCloudVision::on_action_rightMove_triggered()
{
	changeLocationOfObject(2);
}

//�ı��ӽ�
void changeViewOfObject(int index,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr m_currentCloud,PointT p_min, PointT p_max, Ui::PointCloudVisionClass ui)
{
	double point[3];
	pcl::getMinMax3D(*m_currentCloud, p_min, p_max);
	getCenterPoint(m_currentCloud, point);
	double scale = getMaxValue(p_max, p_min);
	outputInDebug(to_string(scale));
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//�����������
	viewer->removeAllCoordinateSystems();
	if (index == 0)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1] + scale * 5, cloudCentroid[2], 0, -0.5, 0, 0, 1, 0);
		ui.textBrowser->append("����ͼ");
	}
	else if (index == 1)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1] - scale * 5, cloudCentroid[2], 0, 0.5, 0, 0, 1, 0);
		ui.textBrowser->append("����ͼ");
	}
	else if (index == 2)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1], cloudCentroid[2] + scale * 5, 0, 0, -0.5, 0, 1, 0);
		ui.textBrowser->append("ǰ��ͼ");
	}
	else if (index == 3)
	{
		viewer->setCameraPosition(cloudCentroid[0], cloudCentroid[1], cloudCentroid[2] - scale * 5, 0, 0, 0.5, 0, 1, 0);
		ui.textBrowser->append("����ͼ");
	}
	else if (index == 4)
	{
		viewer->setCameraPosition(cloudCentroid[0] - scale * 5, cloudCentroid[1], cloudCentroid[2], -0.5, 0, 0, 0, 1, 0);
		ui.textBrowser->append("����ͼ");
	}
	else if (index == 5)
	{
		viewer->setCameraPosition(cloudCentroid[0] + scale * 5, cloudCentroid[1], cloudCentroid[2], 0.5, 0, 0, 0, 1, 0);
		ui.textBrowser->append("����ͼ");
	}
	viewer->addCoordinateSystem(0.5*scale, cloudCentroid[0], cloudCentroid[1], cloudCentroid[2], 0);
	ui.qvtkWidget->update();
	ui.textBrowser->update(); QApplication::processEvents();
}

//����ͼ
void PointCloudVision::on_action_up_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(0, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();

	}
}

//ǰ��ͼ
void PointCloudVision::on_action_front_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(2, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//����ͼ
void PointCloudVision::on_action_left_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(4, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
  ui.textBrowser->update();QApplication::processEvents();


	}
}

//����ͼ
void PointCloudVision::on_action_back_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(3, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
  ui.textBrowser->update();QApplication::processEvents();


	}
}

//����ͼ
void PointCloudVision::on_action_right_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(5, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//����ͼ
void PointCloudVision::on_action_bottom_triggered()
{
	if (!m_currentCloud->empty())
	{
		changeViewOfObject(1, viewer, m_currentCloud, p_min, p_max, ui);
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();
	}
}

//ǰ���
void PointCloudVision::on_action_frontIso_triggered()
{
	if (!m_currentCloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, p_min.y - 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 1, 1, 0);
		ui.qvtkWidget->update();
		ui.textBrowser->append("ǰ���");
		ui.textBrowser->update();QApplication::processEvents();
	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update(); QApplication::processEvents();
	}
}

//�����
void PointCloudVision::on_action_backIso_triggered()
{
	if (!m_currentCloud->empty()) {
		viewer->setCameraPosition(p_max.x + 2 * maxLen, p_max.y + 2 * maxLen, p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), -1, -1, 0);
		ui.qvtkWidget->update();
		ui.textBrowser->append("�����");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("չʾʧ�ܣ�δ���õ���");
		ui.textBrowser->update();QApplication::processEvents();
	}

}

//���õ�����ɫ
void PointCloudVision::on_action_setColor_triggered()
{
	if (!m_currentCloud->empty()) {
		QColor color = QColorDialog::getColor(Qt::white, this, "���õ�����ɫ", QColorDialog::ShowAlphaChannel);
		viewer->removeAllPointClouds();
		pcl::visualization::PointCloudColorHandlerCustom<PointT> singelColor(m_currentCloud, color.red(), color.green(), color.blue());
		viewer->addPointCloud(m_currentCloud, singelColor, "myCloud", 0);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, color.alpha()*1.0 / 255, "myCloud");
		ui.qvtkWidget->update();
		ui.textBrowser->append("������ɫ�ɹ�");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("������ɫʧ�ܣ�δ���õ���");
		ui.textBrowser->update();
		QApplication::processEvents();
	}

}

//���ø߶���Ⱦ
void PointCloudVision::on_action_heightRamp_triggered()
{
	if (!m_currentCloud->empty()) {
		heightRampDlg.show();
	}
	else {
		ui.textBrowser->append("���ø߶���Ⱦ��δ���õ���");
		ui.textBrowser->update();
		QApplication::processEvents();


	}
}

//���и߶���Ⱦ
void PointCloudVision::setHeightRamp(int dir, double height)
{
	
	ui.textBrowser->append("��ʼ�߶���Ⱦ");
	ui.textBrowser->update();
	QApplication::processEvents();
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
	ui.textBrowser->append("�߶���Ⱦ�ɹ�");
	ui.textBrowser->update();


}

//�˲���;
void PointCloudVision::on_action_octree_triggered()
{
	if (!m_currentCloud->empty()) {
		octreeDialog.show();

		//�����˲���;

	}
	else {
		ui.textBrowser->append("�˲���������δ���ص���");
		ui.textBrowser->update(); 
		QApplication::processEvents();
	}
}



//�����˲�����
//���룺�������ơ��ֱ���
//������˲�����
pcl::octree::OctreePointCloudSearch<PointT> createOctree(pcl::PointCloud<PointT>::Ptr cloud, double resolution)
{
	pcl::octree::OctreePointCloudSearch <PointT> tree(resolution);
	tree.setInputCloud(cloud);
	//����߽��
	tree.defineBoundingBox();
	tree.addPointsFromInputCloud();
	return tree;
}


//�˲�����������
//���룺���أ�����������;
void PointCloudVision::octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b)
{
	ui.textBrowser->append("��ʼ����\n");
	ui.textBrowser->update();
	QApplication::processEvents();

	viewer->removeAllPointClouds();
	//�����˲���;
	pcl::octree::OctreePointCloudSearch<PointT> tree = createOctree(m_currentCloud, resolution);
	//����������
	//��ʼ�����㣻
	PointT searchPoint;
	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;
	//��յ��±ꣻ
	std::vector<int> pointIndex;


	//����;
	if (tree.voxelSearch(searchPoint, pointIndex))
	{
	
		for (int i = 0; i < pointIndex.size(); i++)
		{
			
			string s = "��" + to_string(i + 1) + "���ٽ���:\n" + "x=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].x) + "\n" + "y=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].y) + "\n" + "z=" + std::to_string(tree.getInputCloud()->points[pointIndex[i]].z) + "\n";
			/*cout << "��" << i + 1 << "���ٽ��㣺\n"
				<< "x=" << tree.getInputCloud()->points[pointIndex[i]].x << "\n"
				<< "y=" << tree.getInputCloud()->points[pointIndex[i]].y << "\n"
				<< "z=" << tree.getInputCloud()->points[pointIndex[i]].z << "\n"
				<< endl;*/
			ui.textBrowser->append(QString::fromStdString(s));
			ui.textBrowser->update(); 
			QApplication::processEvents();
		} 
		/*-------------------------���ӻ�-------------------------------------------*/
		//Ϊ���ҵ��㴴���µ���;
		pcl::PointCloud<PointT>::Ptr searchCloud(new pcl::PointCloud<PointT>);
		searchCloud->width = m_currentCloud->width;
		searchCloud->height = m_currentCloud->height;
		searchCloud->points.resize(m_currentCloud->points.size());
		for (int i = 0; i < pointIndex.size(); i++) {
			searchCloud->points[i].x = m_currentCloud->points[pointIndex[i]].x;
			searchCloud->points[i].y = m_currentCloud->points[pointIndex[i]].y;
			searchCloud->points[i].z = m_currentCloud->points[pointIndex[i]].z;

		}

		//��ӵ�����
		viewer->addPointCloud(m_currentCloud);

		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> set_searchColor(searchCloud, r, g, b);
		viewer->addPointCloud<PointT>(searchCloud, set_searchColor, "searchPoints");
		ui.qvtkWidget->update();

	}
	else
	{
		ui.textBrowser->append("δ��������");
		ui.textBrowser->update();
		QApplication::processEvents();
	}
}

//�˲���k��������

//��������;
void PointCloudVision::on_action_triangle_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ��������");
		ui.textBrowser->update();QApplication::processEvents();

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

		//setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//�����ӽ�
		viewer->resetCamera();

		//ˢ�´���
		ui.qvtkWidget->update();
		ui.textBrowser->append("�������񻯳ɹ�");
		ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("��������ʧ�ܣ�δ���ص���");
  ui.textBrowser->update();QApplication::processEvents();

	}
	
}
//��������ȡ;
void PointCloudVision::on_action_feature_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ��������ȡ");
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

		//��һ�������е�.pcd�ļ�  �������û��ָ���ͻ��Զ�����
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
		//viewer->resetCamera();
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
		for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
			keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		//�ڹؼ�����ȡNARF������
		std::vector<int> keypoint_indices2;
		keypoint_indices2.resize(keypoint_indices.points.size());
		for (unsigned int i = 0; i < keypoint_indices.size(); ++i)
			keypoint_indices2[i] = keypoint_indices.points[i];					//����NARF�ؼ����������������ʸ����ΪNARF���������������ʹ��

		pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);	//����narf_descriptor���󡣲����˴˶����������ݣ������������������
		narf_descriptor.getParameters().support_size = support_size;			//support_sizeȷ������������ʱ���ǵ������С
		narf_descriptor.getParameters().rotation_invariant = rotation_invariant;    //������ת�����NARF������
		pcl::PointCloud<pcl::Narf36> narf_descriptors;							//����Narf36�ĵ�����������ƶ��󲢽���ʵ�ʼ���
		narf_descriptor.compute(narf_descriptors);								//����������
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//�����ӽ�
		viewer->resetCamera();
		ui.qvtkWidget->update();
		ui.textBrowser->append("��������ȡ�ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("��������ȡʧ�ܣ�δ���ص���");
  ui.textBrowser->update();QApplication::processEvents();

	}
	
}
//���������ָ�;
void PointCloudVision::on_action_grow_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ���������ָ�");
		ui.textBrowser->update();QApplication::processEvents();

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
		for (int i = 0; i < M; i++)
		{
			pcl::PointXYZ p;
			p.x = colored_cloud->points[i].x;
			p.y = colored_cloud->points[i].y;
			p.z = colored_cloud->points[i].z;
			cloud->points.push_back(p);
		}
		cloud->width = 1;
		cloud->height = M;
		setCoordinate(viewer, m_currentCloud);
		maxLen = getMaxValue(p_max, p_min);
		//�����ӽ�
		viewer->resetCamera();
		//ˢ�´���
		ui.qvtkWidget->update();
		ui.textBrowser->append("���������ָ�ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("���������ָ�ʧ�ܣ�δ���ص���");
  ui.textBrowser->update();QApplication::processEvents();

	}

}
//PCA-ICP
void PointCloudVision::on_action_icp_triggered()
{
	ui.textBrowser->append("����δ����");
	ui.textBrowser->update();

}

//SCALE-ICP
void PointCloudVision::on_action_action_scale_icp_triggered()
{
	ui.textBrowser->append("����δ����");
	ui.textBrowser->update();

}

//�����˲�
void PointCloudVision::on_action_3_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ�����˲�");
  ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::VoxelGrid<pcl::PointXYZ> vg;		//�����˲�������
		vg.setInputCloud(m_currentCloud);				//���ô��˲�����
		vg.setLeafSize(1.15f, 1.15f, 1.15f);	//�������ش�С
		vg.filter(*m_currentCloud);			//ִ���˲��������˲������cloud_filtered
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("�����˲��ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("�����˲�ʧ�ܣ�δ���ص���");
  ui.textBrowser->update();QApplication::processEvents();

	}
}

//ֱͨ�˲�;
void PointCloudVision::on_action_4_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼֱͨ�˲�");
  ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::PassThrough<pcl::PointXYZ> pass;	//����ֱͨ�˲�������
		pass.setInputCloud(m_currentCloud);		        //��������ĵ���
		pass.setFilterFieldName("z");           //���ù���ʱ����Ҫ��������ΪZ�ֶ�
		pass.setFilterLimits(-0.1, 10);         //�����ڹ����ֶεķ�Χ
		pass.setFilterLimitsNegative(true);     //���ñ������ǹ��˵��ֶη�Χ�ڵĵ㣬����Ϊtrue��ʾ���˵��ֶη�Χ�ڵĵ�
		pass.filter(*m_currentCloud);		    //ִ���˲�
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("ֱͨ�˲��ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("ֱͨ�˲�ʧ�ܣ�δ���ص����ļ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
}

//ͳ���˲�;
void PointCloudVision::on_action_5_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼͳ���˲�");
  ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //����ͳ���˲������� 
		sor.setInputCloud(m_currentCloud);         			         //��������ĵ���
		sor.setMeanK(50);                 					 //����KNN��kֵ
		sor.setStddevMulThresh(1.0);      				     //���ñ�׼ƫ�����Ϊ1.0
		sor.filter(*m_currentCloud);          			     //ִ���˲�
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("ͳ���˲��ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("ͳ���˲�ʧ�ܣ�δ���ص����ļ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
}

//���Ȳ����˲�;
void PointCloudVision::on_action_6_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ���Ȳ����˲�");
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
		ui.textBrowser->append("���Ȳ����˲��ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("���Ȳ����˲�ʧ�ܣ�δ���ص����ļ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
}

//�뾶�˲�
void PointCloudVision::on_action_7_triggered()
{
	if (!m_currentCloud->empty()) {
		ui.textBrowser->append("��ʼ�뾶�˲�");
  ui.textBrowser->update();QApplication::processEvents();

		viewer->removeAllPointClouds();
		viewer->removeAllCoordinateSystems();
		viewer->removeAllShapes();

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(m_currentCloud);
		outrem.setRadiusSearch(0.001);                     //���ð뾶Ϊ0.1�ķ�Χ�����ٽ���
		outrem.setMinNeighborsInRadius(2);               //���ò�ѯ�������㼯��С��2��ɾ��
		outrem.filter(*m_currentCloud);                  //ִ���˲�
		viewer->addPointCloud(m_currentCloud);
		ui.qvtkWidget->update();
		ui.textBrowser->append("�뾶�˲��ɹ�");
  ui.textBrowser->update();QApplication::processEvents();

	}
	else {
		ui.textBrowser->append("���Ȳ����˲�ʧ�ܣ�δ���ص����ļ�");
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
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//�����������
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//����ƽ�ƾ��󣬲���ʼ��Ϊ��λ��
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
	pcl::compute3DCentroid(*m_currentCloud, cloudCentroid);//�����������
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//����ƽ�ƾ��󣬲���ʼ��Ϊ��λ��
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

/*�󵥸���ķ�����*/
Eigen::MatrixXd computeNormal_(vector<PointT> points) {

	const int num = points.size();//�ٽ��������

	/*���ĵ�����*/
	int x_ = 0;
	int y_ = 0;
	int z_ = 0;

	/*�������ĵ�����*/
	int temp = 0;

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

	Eigen::MatrixXd errorMatrix(3, num);//����������
	Eigen::MatrixXd errorMatrix_trans(3, num);//�����������ת��
	Eigen::EigenSolver<Eigen::MatrixXd> *covarianceMatrix;//Э�������
	Eigen::MatrixXd featureVectros;//��������

	/*��������������*/
	for (int i = 0; i < 3; i++) {

		for (int j = 0; j < num; j++) {

			switch (i) {

			case 0:

				errorMatrix(i,j) = points[j].x;
				break;

			case 1:

				errorMatrix(i, j) = points[j].y;
				break;

			case 2:

				errorMatrix(i, j) = points[j].z;
				break;

			default:
				break;
			}
		}
	}

	/*����ת��*/
	errorMatrix_trans = errorMatrix.transpose();
	/*����Э�������*/
	covarianceMatrix = new  Eigen::EigenSolver<Eigen::MatrixXd>(errorMatrix * errorMatrix_trans);

	/*���������*/
	featureVectros = covarianceMatrix->eigenvectors().real();

	return featureVectros;
}