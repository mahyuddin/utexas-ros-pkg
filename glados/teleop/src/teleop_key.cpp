#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

int kfd = 0;
struct termios cooked, raw;

void init() {
  // put console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  //Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  int flags = fcntl(kfd,F_GETFL,0);
  flags |= O_NONBLOCK;
  fcntl(kfd, F_SETFL, flags);
}

void quit(int) {
  // get back out of raw
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"teleop");
  signal(SIGINT,quit);
  // initialize the terminal
  init();

  char ch;
  int count = 0;
  ros::Time::init();
  ros::Rate r(20); // 20 hz
  while (ros::ok()) {
    //ros::spinOnce();
    //r.sleep();
    ros::spinOnce();
    int ret = read(kfd,&ch,1);
    if (ret < 0) {
      count++;
      continue;
    }

    //ROS_DEBUG("%i %i",ch,count);
    count = 0;
    switch (ch) {
      case KEYCODE_R:
      case 'a':
        ROS_DEBUG("RIGHT");
        break;
      case KEYCODE_L:
      case 'd':
        ROS_DEBUG("LEFT");
        break;
      case KEYCODE_U:
      case 'w':
        ROS_DEBUG("UP");
        break;
      case KEYCODE_D:
      case 's':
        ROS_DEBUG("DOWN");
        break;
      case 'q':
        ROS_DEBUG("q");
        break;
    }
  }

  return 0;
}
