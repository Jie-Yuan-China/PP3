#ifndef SCAN_ANALYSER_H
#define SCAN_ANALYSER_H
#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <std_msgs/Float64.h>
#include "velodyne_pointcloud/calibration.h"
#include "velodyne_msgs/VelodynePacket.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "velodyne_pointcloud/rawdata.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/VelodyneConfigConfig.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/calibration.h"
#include <pcl_conversions/pcl_conversions.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#define DEBUGGING true
class Scan_Analyser
{
public:
    ros::Time received_time_stamp;
    ros::Subscriber time_stamp_sub;
    ros::Subscriber velodyne_scan_sub;
    ros::Publisher transformed_pc_pub;

    //temperary use pcl
    ros::Time time_diff;
    pcl::PointCloud<pcl::PointXYZ> pcl;
    // trajectories from MMS
    std::map<double,std::vector<double>> trajectories;
public:
    Scan_Analyser(ros::NodeHandle nh);
    sensor_msgs::PointCloud2 transform_cloud(const tf::Transform T, const sensor_msgs::PointCloud2 &msg);
    void scanner_callback(const velodyne_msgs::VelodyneScan &scan_msg);
    void time_sync_callback(const std_msgs::Float64 &time_sync_msg);
    void convert_3d_lidar(velodyne_msgs::VelodyneScan v_scan,nav_msgs::Odometry gps_pose);
    const std::vector<std::string> split(std::string& s, const std::string chars);
    void read_MMS_trajectory(std::string path);
    std::vector<double> interpolate_pose(double &time);
    double my_stod(std::string& s);
    std::pair<double,double> closest(std::vector<double> const& vec, double value);
    tf::Transform generate_tf_matrix(std::vector<double> &params);
   virtual ~Scan_Analyser();
};

#endif // SCAN_ANALYSER_H
