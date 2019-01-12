#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "time_sync_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
