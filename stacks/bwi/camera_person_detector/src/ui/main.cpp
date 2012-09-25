#include "DetectorViewer.h"
#include <QtGui>
#include <QApplication>

int main(int argc, char **argv) {
  
  QApplication app(argc, argv);
  camera_person_detector::DetectorViewer w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  app.connect(&w, SIGNAL(rosShutdown()), &app, SLOT(quit()));
  app.connect(w.ui.cbxRegisterAll, SIGNAL(stateChanged(int)), &w, SLOT(setRegisterAll(int)));
  app.connect(w.ui.cbxRegisterPerson, SIGNAL(stateChanged(int)), &w, SLOT(setRegisterPerson(int)));
  int result = app.exec();

	return result;
}
