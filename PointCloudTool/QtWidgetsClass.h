#pragma once

#include <QDialog>
#include "ui_QtWidgetsClass.h"

class QtWidgetsClass : public QDialog
{
	Q_OBJECT

public:
	QtWidgetsClass(QWidget *parent = Q_NULLPTR);
	~QtWidgetsClass();

signals:
	void filter_dialog_function(int times);
private:
	Ui::QtWidgetsClass ui;

private slots:
	void on_pushButton_clicked();
};
