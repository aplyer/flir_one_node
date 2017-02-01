#include <signal.h>
#include "driver_flir.h"

void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  ROS_ERROR("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_flir_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  ros::NodeHandle camera_nh("~");
  signal(SIGSEGV, &sigsegv_handler);
  driver_flir::DriverFlir dvr(node, priv_nh, camera_nh);
  ros::AsyncSpinner spinner(4);
  dvr.setup();
  spinner.start();
  while (node.ok() && dvr.ok()){
    dvr.poll();
  }

  ros::waitForShutdown();
  dvr.shutdown();

  return 0;
}
