/**
 * @file /include/bwi_person_controller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef bwi_person_controller_QNODE_HPP_
#define bwi_person_controller_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace bwi_person_controller {

  /*****************************************************************************
   ** Class
   *****************************************************************************/

  class QNode : public QThread {
    Q_OBJECT
    public:
      QNode(int argc, char** argv );
      virtual ~QNode();
      bool init();
      bool init(const std::string &master_url, const std::string &host_url);
      void run();

    signals:
      void rosShutdown();

    private:
      int init_argc;
      char** init_argv;
  };

}  // namespace bwi_person_controller

#endif /* bwi_person_controller_QNODE_HPP_ */
