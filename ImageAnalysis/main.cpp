#include "imageanalysis.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ImageAnalysis w;
	w.show();
	return a.exec();
}
