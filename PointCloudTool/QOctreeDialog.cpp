#include "QOctreeDialog.h"
#include <QColorDialog>

QOctreeDialog::QOctreeDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowModality(Qt::ApplicationModal);
}

QOctreeDialog::~QOctreeDialog()
{
}

//体素近邻搜索
void QOctreeDialog::on_btn_vSearch_clicked()
{
	double resolution = ui.input_resolution->value();
	double x = ui.input_X->value();
	double y = ui.input_Y->value();
	double z = ui.input_Z->value();
	emit octree_vsearch(resolution, x, y, z, r, g, b);
	close();
}

//k邻域搜索
void QOctreeDialog::on_btn_kSearch_clicked()
{
}

//半径邻域搜索
void QOctreeDialog::on_btn_rSearch_clicked()
{
}

//设置颜色
void QOctreeDialog::on_btn_color_clicked()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "设置搜索点颜色", QColorDialog::ShowAlphaChannel);
	color.getRgb(&r, &g, &b, nullptr);
}
