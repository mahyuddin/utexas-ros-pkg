/**
 * @file /include/bwi_person_controller/main_window.hpp
 *
 * @brief Qt based gui for bwi_person_controller.
 *
 * @date November 2010
 **/
#ifndef bwi_person_controller_MAIN_WINDOW_H
#define bwi_person_controller_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace bwi_person_controller {

  /*****************************************************************************
   ** Interface [MainWindow]
   *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
      MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

      void closeEvent(QCloseEvent *event); // Overloaded function
      void keyPressEvent(QKeyEvent* pEvent); // Overloaded function
      bool navigate(double x, double y, std::string level, std::string& error);
      void move(double x, double theta, QToolButton &button);

    public slots:
      /******************************************
       ** Auto-connections (connectSlotsByName())
       *******************************************/
      void on_leftButton_clicked();
      void on_rightButton_clicked();
      void on_forwardButton_clicked();
      void on_backwardButton_clicked();
      void on_stopButton_clicked();

      void on_danaButton_clicked();
      void on_piyushButton_clicked();
      void on_ensButton_clicked();
      void on_samButton_clicked();

    private:
      Ui::MainWindowDesign ui;
      QNode qnode;
      double current_speed;
      double current_angular_speed;
  };

}  // namespace bwi_person_controller

#endif // bwi_person_controller_MAIN_WINDOW_H
