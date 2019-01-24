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
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#define DEBUGGING false
class Scan_Analyser
{
public:
    ros::Time received_time_stamp;
    ros::Subscriber time_stamp_sub;
    ros::Subscriber velodyne_scan_sub;// for the pointcloud sub, no accurate time info from header.stamp.
    // @ref: http://docs.pointclouds.org/1.8.1/structpcl_1_1_p_c_l_header.html#a7ccf28ecce53332cd572d1ba982a4579
    ros::Subscriber raw_veloscan_sub; // used to get the time undirectly, rather than ros::Time::now();
    ros::Publisher transformed_pc_pub;
    pcl::PLYWriter writer;
    std::vector<double> time_stamps_mms;
    //temperary use pcl
    ros::Time actual_timestamp; // get time from header of velodyneScan msg
    ros::Time last_timestamp;
    uint32_t traj_idx;// to improve the efficiency of searching traj
    // trajectories from MMS
    std::map<double,std::vector<double>> trajectories;
public:
    Scan_Analyser(ros::NodeHandle nh);
    sensor_msgs::PointCloud2 transform_cloud(const tf::Transform T, const sensor_msgs::PointCloud2 &msg);
    void scanner_callback(const velodyne_rawdata::VPointCloud::ConstPtr& inMsg);
    void time_sync_callback(const std_msgs::Float64 &time_sync_msg);
    const std::vector<std::string> split(std::string& s, const std::string chars);
    void load_MMS_trajectory(std::string path);
    std::vector<double> interpolate_pose(double &time);
    double my_stod(std::string& s);
    std::pair<double,double> closest(std::vector<double> const& vec, double value);
    tf::Transform generate_tf_matrix(std::vector<double> &params);
    virtual ~Scan_Analyser();
};

#endif // SCAN_ANALYSER_H
