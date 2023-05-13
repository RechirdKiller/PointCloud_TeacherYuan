#pragma once

#include <QDialog>
#include "ui_QOctrTree_my.h"

class QOctrTree_my : public QDialog
{
	Q_OBJECT

public:
	QOctrTree_my(QWidget *parent = Q_NULLPTR);
	~QOctrTree_my();

private slots:
	void on_btn_SearchFirst_clicked();

	void on_btn_SearchSecond_clicked();

	void on_btn_SearchThird_clicked();

signals:
	void searchPointByOctreeCreatedByDepth(int depth,int pointNum,double radius,int flag,double x, double y, double z);

private:
	Ui::QOctrTree_my ui;

	int depth;
	int pointNum;
	double radius;
	int flag;
	double x;
	double y;
	double z;
};
