#pragma once

#include <QDialog>
#include "ui_QOctreeDialog.h"

class QOctreeDialog : public QDialog
{
	Q_OBJECT

public:
	QOctreeDialog(QWidget *parent = Q_NULLPTR);
	~QOctreeDialog();

private slots:
	//ÉèÖÃÑÕÉ«
	void on_btn_color_clicked();

	//ÌåËØËÑË÷
	void on_btn_vSearch_clicked();

	//kÁÚÓòËÑË÷
	void on_btn_kSearch_clicked();

	//°ë¾¶ÁÚÓòËÑË÷
	void on_btn_rSearch_clicked();


signals:
	void octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b);

private:
	Ui::QOctreeDialog ui;

	int r;
	int g;
	int b;
};
