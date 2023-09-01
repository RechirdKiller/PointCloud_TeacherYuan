#include "QtWidgetsClass.h"

QtWidgetsClass::QtWidgetsClass(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), ui.spinBox, SLOT(setValue(int)));
	connect(ui.spinBox, SIGNAL(valueChanged(int)), ui.horizontalSlider, SLOT(setValue(int)));
}

QtWidgetsClass::~QtWidgetsClass()
{
}

void QtWidgetsClass::on_pushButton_clicked()
{
	int times = ui.horizontalSlider->value();
	emit filter_dialog_function(times);
	close();
}
