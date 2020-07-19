#include "curveAndSurface.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	curveAndSurface w;
	w.resize(700, 400);
	w.show();
	return a.exec();
}
