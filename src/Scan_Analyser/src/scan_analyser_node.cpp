#include <ros/ros.h>
#include <scan_analyser.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_analyser_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
