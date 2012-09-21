/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/bwi_person_controller/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bwi_person_controller {

  using namespace Qt;

  /*****************************************************************************
   ** Implementation [MainWindow]
   *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc,argv), current_speed(0.5), current_angular_speed(0.5)
  {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    move(0,0,*(ui.stopButton));
  }

  MainWindow::~MainWindow() {}

  void MainWindow::navigate(double x, double y, uint32_t level) {
    // perhaps set some sort of feedback callback here?
    std::string error;
    qnode.navigate(x,y,level,error);
    // if (success) { // Need to call using callback
    //   ui.statusbar->showMessage("Navigation successful!!", 5000);
    // } else {
    //   ui.statusbar->showMessage(QString(error.c_str()), 5000);
    // }
  }

  void MainWindow::move(double x, double theta, QToolButton &button) {
    ui.leftButton->setDown(false);
    ui.rightButton->setDown(false);
    ui.forwardButton->setDown(false);
    ui.backwardButton->setDown(false);
    ui.stopButton->setDown(false);
    button.setDown(true);
    qnode.move(x, theta);
  }
  
  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/
  void MainWindow::on_leftButton_clicked() {
    move(0, current_angular_speed, *(ui.leftButton));
  }
  void MainWindow::on_rightButton_clicked() {
    move(0, -current_angular_speed, *(ui.rightButton));
  }
  void MainWindow::on_forwardButton_clicked() {
    move(current_speed, 0, *(ui.forwardButton));
  }
  void MainWindow::on_backwardButton_clicked() {
    move(-current_speed, 0, *(ui.backwardButton));
  }
  void MainWindow::on_stopButton_clicked() {
    move(0, 0, *(ui.stopButton));
  }
  void MainWindow::on_danaButton_clicked() {
    navigate(-4,9,2); 
  }
  void MainWindow::on_piyushButton_clicked() {
    navigate(-7,1,2); 
  }
  void MainWindow::on_samButton_clicked() {
    navigate(7,22,2); 
  }
  void MainWindow::on_ensButton_clicked() {
    navigate(1,1,2); 
  }

  void MainWindow::closeEvent(QCloseEvent *event)
  {
    QMainWindow::closeEvent(event);
  }

  void MainWindow::keyPressEvent(QKeyEvent *event) {
    switch(event->key()) {
      case Qt::Key_I:
        on_forwardButton_clicked();
        break;
      case Qt::Key_K:
        on_stopButton_clicked();
        break;
      case Qt::Key_Comma:
        on_backwardButton_clicked();
        break;
      case Qt::Key_J:
        on_leftButton_clicked();
        break;
      case Qt::Key_L:
        on_rightButton_clicked();
        break;
    }
  }

}  // namespace bwi_person_controller

