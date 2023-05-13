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
	//������ɫ
	void on_btn_color_clicked();

	//��������
	void on_btn_vSearch_clicked();

	//k��������
	void on_btn_kSearch_clicked();

	//�뾶��������
	void on_btn_rSearch_clicked();

	//��������1
	void on_btn_sSearch_clicked();

	//k��������1
	void on_btn_mSearch_clicked();

	//�뾶��������1
	void on_btn_cSearch_clicked();

signals:
	void octree_vsearch(double resolution, double x, double y, double z, int r, int g, int b);
	void octree_vsearch_zz(double resolution, double x, double y, double z, int r, int g, int b,int flag);

private:
	Ui::QOctreeDialog ui;

	int r;
	int g;
	int b;
};
