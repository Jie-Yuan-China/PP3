#include <ros/ros.h>
#include <string.h>
#include "scan_analyser.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_analyser_node");
  ros::NodeHandle nh;
  Scan_Analyser ana =  Scan_Analyser(nh);
  ros::spin();


}
