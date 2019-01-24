#include "scan_analyser.h"
#include "nav_msgs/Odometry.h"
#include "velodyne_pointcloud/pointcloudXYZIR.h"
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud2.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

#include "pcl_ros/point_cloud.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
extern "C" {
#include "swiftnav/gnss_time.h"
}
#include "velodyne_pointcloud/rawdata.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
Scan_Analyser::Scan_Analyser(ros::NodeHandle nh)
{   //diff time for debuging
    time_t traj_start_time = gps2time(new gps_time_t {308729.418746,2037});
    double time_diff_traj = 1548250178.232908 - traj_start_time;
    ROS_INFO("%f has passed since the measurement.",time_diff_traj);

    //load the traj data
    std::string path = "/home/jie/catkin_ws/data/trajectory_calenbergerneustadt.txt";
    traj_idx = 0;
    load_MMS_trajectory( path );
    // add the subscriber and publisher
    time_stamp_sub= nh.subscribe("/trigger_timestamps", 1, &Scan_Analyser::time_sync_callback,this);
    velodyne_scan_sub = nh.subscribe("/velodyne_points",1,&Scan_Analyser::scanner_callback,this);
    transformed_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/velo_transformed",1);

}
Scan_Analyser::~Scan_Analyser(){

}

void Scan_Analyser::scanner_callback(const velodyne_rawdata::VPointCloud::ConstPtr& inMsg){
    if (!actual_timestamp.isZero() && !last_timestamp.isZero()){
        ROS_INFO("start processing current cloud");
        ros::Time begin_stamp = ros::Time::now();

        std::cout<<"size of points of this msg: "<<inMsg->points.size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZI> current_cloud;
        std::map<uint16_t,tf::Transform> transform_map;
        for(uint16_t i = 0; i < 36000; i++){
            double current_scan_time = last_timestamp.toSec() + i * (actual_timestamp.toSec()-last_timestamp.toSec())/36000;
            std::vector<double> params = interpolate_pose(current_scan_time);
            tf::Transform T = generate_tf_matrix(params);
            transform_map[i] = T;
        }
        for(auto pt:inMsg->points){
            tf::Vector3 pt_transformed = transform_map[pt.azimuth] * tf::Vector3(pt.x,pt.y,pt.z);
            pcl::PointXYZI target_pt;
            target_pt.x = pt_transformed.getX();
            target_pt.y = pt_transformed.getY();
            target_pt.z = pt_transformed.getZ();
            target_pt.intensity = pt.intensity;
            current_cloud.push_back(target_pt);
        }
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(current_cloud,output_msg);
        output_msg.header.frame_id = "velodyne";
        transformed_pc_pub.publish(output_msg);
        ROS_INFO("//////Processing used %f seconds",ros::Time::now().toSec()-begin_stamp.toSec());
        if (writer.write("/home/jie/testsave.ply", current_cloud, false, false) != 0) {
          ROS_ERROR("Something went wrong when trying to write the point cloud file.");
          return;
    }
        ROS_INFO("####################Split line####################");

    }

}
void Scan_Analyser::time_sync_callback(const std_msgs::Float64 &time_sync_msg){
    if (!actual_timestamp.isZero()){
        last_timestamp = actual_timestamp;
    }
    actual_timestamp.fromSec(time_sync_msg.data);

}
std::vector<double> Scan_Analyser::interpolate_pose(double &time){
    std::vector<double> params;

    std::pair <double,double> time_pair = closest(time_stamps_mms, time);
    std::vector<double> param_before = trajectories[time_pair.first];
    std::vector<double> param_after = trajectories[time_pair.second];
    if (param_before.empty() || param_after.empty()){
        std::cerr<<"found no param"<<std::endl;
    }else{
        if (DEBUGGING){
            ROS_INFO("param_before: %f",param_before.at(0));
            ROS_INFO("param_before: %f",param_before.at(1));
            ROS_INFO("param_before: %f",param_before.at(2));
            ROS_INFO("param_before: %f",param_before.at(3));
            ROS_INFO("param_before: %f",param_before.at(4));
            ROS_INFO("param_before: %f",param_before.at(5));
            ROS_INFO("***********************");
            ROS_INFO("param_after: %f",param_after.at(0));
            ROS_INFO("param_after: %f",param_after.at(1));
            ROS_INFO("param_after: %f",param_after.at(2));
            ROS_INFO("param_after: %f",param_after.at(3));
            ROS_INFO("param_after: %f",param_after.at(4));
            ROS_INFO("param_after: %f",param_after.at(5));
        }
    }
    double time_diff_before = time - time_pair.first;
    double time_diff_after = time_pair.second - time;
    double ratio = time_diff_before/(time_diff_after+time_diff_before);
    if(DEBUGGING){
        ROS_INFO("Time difference should be %f",time_pair.second - time_pair.first);
    }
    for(int i = 0; i < param_before.size(); i++){
        params.push_back(param_before[i] + ratio * (param_after[i] - param_before[i]));
    }
    if (DEBUGGING){
        ROS_INFO("time_diff_before must positive: %f",time_diff_before);
        ROS_INFO("time_diff_after must positive: %f",time_diff_after);
        ROS_INFO("ratio must positive: %f",ratio);
        ROS_INFO("param_interpolated: %f",params.at(0));
        ROS_INFO("param_interpolated: %f",params.at(1));
        ROS_INFO("param_interpolated: %f",params.at(2));
        ROS_INFO("param_interpolated: %f",params.at(3));
        ROS_INFO("param_interpolated: %f",params.at(4));
        ROS_INFO("param_interpolated: %f",params.at(5));
        ROS_INFO("***********************");

    }
    return params;

}
void Scan_Analyser::load_MMS_trajectory(std::string path){//DONE confirmed right
    std::ifstream inFile;

    inFile.open(path);
    if (!inFile) {
        std::cerr << "Unable to open file ";
        exit(1);   // call system to stop
    }
    std::string line;
    while(std::getline(inFile,line)){
        std::vector<std::string> tmp = split(line, ",");
        if (tmp.at(0) != "Time[s]"){
            std::vector<double> params;
            std::vector<std::string> ::iterator param_it;
            for (param_it = tmp.begin() + 1; param_it != tmp.end();param_it++){
                params.push_back(my_stod(*param_it));
            }
            //////////////////////////////////////////////////////////
            time_t t = gps2time(new gps_time_t {my_stod(*tmp.begin()),2037});
            //////////////////////////////////////////////////////////
            trajectories[t] = params;
            time_stamps_mms.push_back(t);
        }
    }
    std::cout<<"finished load trajectory"<<std::endl;
}

std::pair<double,double> Scan_Analyser::closest(std::vector<double> const& vec, double value) {

    auto const it_before = std::lower_bound(vec.begin() + traj_idx, vec.end(), value);
    auto const it_after = std::upper_bound(vec.begin() + traj_idx, vec.end(), value);
    if (it_before == vec.end() || it_after == vec.end() ) { return std::make_pair(-1.0,-1.0); }
    if (DEBUGGING){
        std::cout << "upper_bound at position " << (it_after - vec.begin()) << '\n';
        std::cout << "lower_bound at position " << (it_before- vec.begin() - 1) << '\n';
        std::cout<<"time_before found:"<<std::setprecision(15)<<*(it_before - 1)<<", ";
        std::cout<<"time wanted:"<<std::setprecision(15)<<value<<", ";
        std::cout<<"time_after found:" <<std::setprecision(15)<<*it_after<<std::endl;
    }
    traj_idx = it_before - vec.begin();
    return std::make_pair(*(it_before - 1),*it_after);//////
}

double Scan_Analyser::my_stod(std::string& s)
{
    std::istringstream stream(s);
    double i;
    stream >> i;
    return i;
}
const std::vector<std::string> Scan_Analyser::split(std::string& s, const std::string chars){

    std::string buff { "" };
    std::vector<std::string> v;
    for (auto n : s) {
        bool split = false, del = false;
        for (auto c : chars) {
            if (n == c) {
                del = true;
                if (buff != "") {
                    split = true;
                }
            }
        }
        if (split) {
            split = false;
            v.push_back(buff);
            buff = "";
            continue;
        }
        if (!split && !del)
            buff += n;
    }
    if (buff != "")
        v.push_back(buff);
    return v;
}
tf::Transform Scan_Analyser::generate_tf_matrix(std::vector<double> &param){
    tf::Transform transmatrix = tf::Transform::getIdentity();
    tf::Quaternion quat = tf::Quaternion::getIdentity();

    quat.setRPY(param[3] * M_PI / 180,param[4]* M_PI / 180,param[5]* M_PI / 180);
    transmatrix.setRotation(quat);
    if (DEBUGGING){
        transmatrix.setOrigin(tf::Vector3(param[0]- 544432.5269 ,param[1]- 5806532.7835 ,param[2]- 92.9471 ));
    }else{
        transmatrix.setOrigin(tf::Vector3(param[0]- 544432.5269,param[1]- 5806532.7835,param[2]- 92.9471));
    }

    return transmatrix;
}
sensor_msgs::PointCloud2 Scan_Analyser::transform_cloud(const tf::Transform T, const sensor_msgs::PointCloud2 &msg){
    //TODO transform the cloud using T
    sensor_msgs::PointCloud2 transformed_pc;
    pcl_ros::transformPointCloud("velodyne", T, msg, transformed_pc);
    return transformed_pc;

}


//void Scan_Analyser::convert_3d_lidar(
//        const velodyne_msgs::VelodyneScan v_scan,
//        const nav_msgs::Odometry gps_pose)
//{
//    pcl::PointCloud<pcl::PointXYZI> current_cloud_3d;
//    current_cloud_3d.clear();
//    pcl::PointXYZI point;
//    velodyne_rawdata::RawData data;
//    data.setParameters(0.9,130.0,0.0,6.28);// TODO Change the parameters,
//    data.setupOffline("file adresss",130.0,0.9);// TODO set calibration file, as well as max range, min range.
//    velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());
//    outMsg->header.stamp = pcl_conversions::toPCL(v_scan.header).stamp;
////    outMsg->header.frame_id = config_.frame_id;
////    outMsg->height = 1;
//    velodyne_pointcloud::PointcloudXYZIR v_point_cloud;
//    for(size_t i=0;i<v_scan.packets.size();++i)
//    {
//        v_point_cloud.pc->points.clear();
//        v_point_cloud.pc->width = 0;
//        v_point_cloud.pc->height = 1;
//        v_point_cloud.pc->header.stamp = pcl_conversions::toPCL(v_scan.header).stamp;
//        v_point_cloud.pc->header.frame_id = v_scan.header.frame_id;

//        //v_point_cloud.pc->points.reserve(v_scan->packets.size() * data.scansPerPacket());
//        //pcl::PointCloud<velodyne_pointcloud::PointXYZIR> v_point_cloud;
//        data.unpack(v_scan.packets[i],v_point_cloud);
//        for(int j=0;j<v_point_cloud.pc->size();j++)
//        {
//            point.x=v_point_cloud.pc->points[j].x;
//            point.y=v_point_cloud.pc->points[j].y;
//            point.z=v_point_cloud.pc->points[j].z;
//            point.intensity=v_point_cloud.pc->points[j].intensity;
//            current_cloud_3d.push_back(point);
//            std::cout<<"transform to PCL done"<<std::endl;
//        }
//    }
//}
