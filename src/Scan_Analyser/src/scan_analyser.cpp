#include "scan_analyser.h"

Scan_Analyser::Scan_Analyser(ros::NodeHandle nh)
{
    time_stamp_sub= nh.subscribe("/trigger_timestamps", 1, &Scan_Analyser::time_sync_callback,this);
    velodyne_scan_sub = nh.subscribe("/velodyne_packets",1,&Scan_Analyser::scanner_callback,this);;
}
Scan_Analyser::~Scan_Analyser(){

}
// TODO interpolate the scanpose
void Scan_Analyser::scanner_callback(const velodyne_msgs::VelodyneScan &scan_msg){

}
void Scan_Analyser::time_sync_callback(const std_msgs::Float64 &time_sync_msg){

}
