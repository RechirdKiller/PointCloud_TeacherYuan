#include "PointCloudVision.h"
#include <QtWidgets/QApplication>
#include <vtkoutputwindow.h>



int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);
	QApplication a(argc, argv);
	PointCloudVision w;
	w.show();
	return a.exec();
}
