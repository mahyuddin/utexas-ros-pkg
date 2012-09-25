#include "DetectorViewer.h"
#include <QtGui>
#include <QApplication>

int main(int argc, char **argv) {
  
  QApplication app(argc, argv);
  camera_person_detector::DetectorViewer w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  app.connect(&w, SIGNAL(rosShutdown()), &app, SLOT(quit()));
  int result = app.exec();

	return result;
}
