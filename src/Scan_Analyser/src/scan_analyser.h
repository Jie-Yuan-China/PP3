#ifndef SCAN_ANALYSER_H
#define SCAN_ANALYSER_H
#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <std_msgs/Float64.h>
class Scan_Analyser
{
public:
    ros::Subscriber time_stamp_sub;
    ros::Subscriber velodyne_scan_sub;
public:
    Scan_Analyser(ros::NodeHandle nh);
    void scanner_callback(const velodyne_msgs::VelodyneScan &scan_msg);
    void time_sync_callback(const std_msgs::Float64 &time_sync_msg);
    ~Scan_Analyser();
};

#endif // SCAN_ANALYSER_H
