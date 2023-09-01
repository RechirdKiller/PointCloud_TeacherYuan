#pragma once

#include<qwidget.h>
#include <QDialog>
#include "ui_segmentation_dragon.h"

class segmentation_dragon : public QDialog
{
	Q_OBJECT

public:
	segmentation_dragon(QWidget *parent = Q_NULLPTR);
	~segmentation_dragon();

private slots:
	void on_button_plane_clicked();

	void on_pushButton_3_clicked();

	void on_button_mian_clicked();

signals:
	void segmentation_calculate_by_choice(int k_choice, double DisThre, int minSize, double normalWeight, int maxite, int maxError);

private:
	
	double DisThre;
	int minSize;
	double normalWeight;
	int maxite;
	int maxError;
	Ui::segmentation_dragon ui;
};
