#include "QOctrTree_my.h"

QOctrTree_my::QOctrTree_my(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	connect(ui.slider, SIGNAL(valueChanged(int)), ui.spinBox_2, SLOT(setValue(int)));
	connect(ui.spinBox_2, SIGNAL(valueChanged(int)), ui.slider, SLOT(setValue(int)));
}

QOctrTree_my::~QOctrTree_my()
{
}

void QOctrTree_my::on_btn_SearchFirst_clicked()
{
	depth = ui.slider->value();
	pointNum = ui.spinBox->value();
	radius = ui.doubleSpinBox_4->value();
	x = ui.doubleSpinBox->value();
	y = ui.doubleSpinBox_2->value();
	z = ui.doubleSpinBox_3->value();
	emit searchPointByOctreeCreatedByDepth(depth,pointNum,radius,0,x,y,z);
	close();
}

void QOctrTree_my::on_btn_SearchSecond_clicked()
{
	depth = ui.slider->value();
	pointNum = ui.spinBox->value();
	radius = ui.doubleSpinBox_4->value();
	x = ui.doubleSpinBox->value();
	y = ui.doubleSpinBox_2->value();
	z = ui.doubleSpinBox_3->value();
	emit searchPointByOctreeCreatedByDepth(depth, pointNum, radius, 1, x, y, z);
	close();
}

void QOctrTree_my:: on_btn_SearchThird_clicked()
{
	depth = ui.slider->value();
	pointNum = ui.spinBox->value();
	radius = ui.doubleSpinBox_4->value();
	x = ui.doubleSpinBox->value();
	y = ui.doubleSpinBox_2->value();
	z = ui.doubleSpinBox_3->value();
	emit searchPointByOctreeCreatedByDepth(depth, pointNum, radius, 2, x, y, z);
	close();
}