#include "segmentation_dragon.h"

segmentation_dragon::segmentation_dragon(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), ui.spinBox, SLOT(setValue(int)));
	connect(ui.spinBox, SIGNAL(valueChanged(int)), ui.horizontalSlider, SLOT(setValue(int)));

	connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)), ui.spinBox_2, SLOT(setValue(int)));
	connect(ui.spinBox_2, SIGNAL(valueChanged(int)), ui.horizontalSlider_2, SLOT(setValue(int)));

	connect(ui.horizontalSlider_3, SIGNAL(valueChanged(int)), ui.spinBox_3, SLOT(setValue(int)));
	connect(ui.spinBox_3, SIGNAL(valueChanged(int)), ui.horizontalSlider_3, SLOT(setValue(int)));

	connect(ui.horizontalSlider_4, SIGNAL(valueChanged(int)), ui.spinBox_4, SLOT(setValue(int)));
	connect(ui.spinBox_4, SIGNAL(valueChanged(int)), ui.horizontalSlider_4, SLOT(setValue(int)));

	connect(ui.horizontalSlider_5, SIGNAL(valueChanged(int)), ui.spinBox_5, SLOT(setValue(int)));
	connect(ui.spinBox_5, SIGNAL(valueChanged(int)), ui.horizontalSlider_5, SLOT(setValue(int)));
}

segmentation_dragon::~segmentation_dragon()
{
}

void segmentation_dragon::on_button_plane_clicked()
{
	DisThre = ((double)ui.spinBox->value())/100;
	minSize = ui.spinBox_2->value();
	normalWeight = ((double)ui.spinBox_4->value())/100;
	maxite = ui.spinBox_3->value();
	maxError = ui.spinBox_5->value(); ;
	emit segmentation_calculate_by_choice(1, DisThre, minSize, normalWeight, maxite, maxError);
	close();
}

void segmentation_dragon::on_pushButton_3_clicked()
{
	DisThre = ((double)ui.spinBox->value()) / 100;
	minSize = ui.spinBox_2->value();
	normalWeight = ((double)ui.spinBox_4->value()) / 100;
	maxite = ui.spinBox_3->value();
	maxError = ui.spinBox_5->value();
	emit segmentation_calculate_by_choice(2, DisThre, minSize, normalWeight, maxite, maxError);
	close();
}

void segmentation_dragon::on_button_mian_clicked()
{
	DisThre = ((double)ui.spinBox->value()) / 100;
	minSize = ui.spinBox_2->value();
	normalWeight = ((double)ui.spinBox_4->value()) / 100;
	maxite = ui.spinBox_3->value();;
	maxError = ui.spinBox_5->value(); ;
	emit segmentation_calculate_by_choice(0, DisThre, minSize, normalWeight, maxite, maxError);
	close();
}