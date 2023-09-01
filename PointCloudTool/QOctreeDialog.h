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
	void octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b,int orderNum);
	void octree_kdtSearch(double resolution, double x, double y, double z, int r, int g, int b,int pointNum, int orderNum);
	void octree_radiusTreeSearsch(double resolution, double x, double y, double z, int r, int g, int b,int radiusAccount, int orderNum);
	void octree_vsearch_zz(double resolution, double x, double y, double z, int r, int g, int b,int flag);

private:
	Ui::QOctreeDialog ui;

	int r;
	int g;
	int b;
};
