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
#include <boost/thread/mutex.hpp>

#include <bwi_msgs/NavigatePerson.h>

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
      void navigate(double x, double y, uint32_t level, std::string& error);
      void move(double x, double theta);

    signals:
      void rosShutdown();

    private:
      int init_argc;
      char** init_argv;
      ros::ServiceClient navigate_client;
      ros::Publisher vel_pub;
      int person_id;
      bool initialized;
      std::vector<bwi_msgs::NavigatePerson> navigation_queue;
      boost::mutex navigation_mutex;
  };

}  // namespace bwi_person_controller

#endif /* bwi_person_controller_QNODE_HPP_ */
