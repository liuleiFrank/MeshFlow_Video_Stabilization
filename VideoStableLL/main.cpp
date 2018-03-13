#include "videostablell.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	VideoStableLL w;
	w.show();
	return a.exec();
}
