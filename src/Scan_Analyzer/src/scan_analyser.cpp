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

Scan_Analyser::Scan_Analyser(ros::NodeHandle nh)
{   //diff time for debuging
    time_t traj_start_time = gps2time(new gps_time_t {308709.930850,2035});
    time_diff_traj.fromSec(ros::Time::now().toSec() - traj_start_time);
    time_diff_ros.fromSec(ros::Time::now().toSec() - 1547041518.55);
    ROS_INFO("%f has passed since the measurement.",time_diff_traj.toSec());

    //load the traj data
    std::string path = "/home/jie/catkin_ws/data/trajectory (another copy).txt";
    read_MMS_trajectory( path );
    // add the subscriber and publisher
    time_stamp_sub= nh.subscribe("/trigger_timestamps", 1, &Scan_Analyser::time_sync_callback,this);
    velodyne_scan_sub = nh.subscribe("/velodyne_points",50,&Scan_Analyser::scanner_callback,this);
    transformed_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/velo_transformed",1);
    raw_veloscan_sub = nh.subscribe("/velodyne_packets",1,&Scan_Analyser::indirect_time_callback,this);

}
Scan_Analyser::~Scan_Analyser(){

}
void Scan_Analyser::indirect_time_callback(const velodyne_msgs::VelodyneScan& msg){
    ROS_INFO("Received time from the VelodyneScan Msg: %f", msg.header.stamp.toSec());
    actual_time = msg.header.stamp;
}

void Scan_Analyser::scanner_callback(const velodyne_rawdata::VPointCloud::ConstPtr& inMsg){

    if (received_time_stamp.isValid() && received_time_stamp.toSec() > 0.0){
    // in case of lost data, map every points with the azimuth
    std::cout<<"size of points of this msg: "<<inMsg->points.size()<<std::endl;
    ros::Time finished_timestamp = actual_time;
    gps_time_t time = time2gps_t(finished_timestamp.toSec());
    ROS_INFO("time receiving the message: %f , consrespond to GPS time %f", finished_timestamp.toSec(), time.tow);

    ros::Time beginning_timestamp = finished_timestamp - ros::Duration(0.1);//temp setting
    pcl::PointCloud<pcl::PointXYZI> current_cloud;
    for(uint16_t i = 0; i < 36000; i++){
        pcl::PointCloud<pcl::PointXYZI> sliced_cloud;
        for(auto pt:inMsg->points){
            if(pt.azimuth == i){
                pcl::PointXYZI target_pt;
                target_pt.x = pt.x;
                target_pt.y = pt.y;
                target_pt.z = pt.z;
                target_pt.intensity = pt.intensity;
//                ROS_INFO("Pt info %f,%f,%f,%f", pt.x,pt.y,pt.z,pt.intensity);
                sliced_cloud.push_back(target_pt);
            }
        }
        double current_scan_time = beginning_timestamp.toSec() + i * (finished_timestamp.toSec()-beginning_timestamp.toSec())/36000;
//        std::cout<<"current time needed:"<<current_scan_time<<std::endl;
        std::vector<double> params = interpolate_pose(current_scan_time);
        tf::Transform T = generate_tf_matrix(params);
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        pcl_ros::transformPointCloud(sliced_cloud,transformed_cloud,T);
        current_cloud += transformed_cloud;
    }
    writer.write("test_save.pcd",current_cloud);
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(current_cloud,output_msg);
    output_msg.header.frame_id = "velodyne";
    transformed_pc_pub.publish(output_msg);
    ROS_INFO("####################Split line####################");

    // @TODO add a interpolated timestamp to every pointcloud using the timestamp from the header as reference,
    // then take all points with same azimuth, cal the time when the laserscanner scans at the specific azimuth, then output as the pointcloud.
    // think about how to debug this part. then how to improve the speed.

//    ROS_INFO("received_time_stamp:%f",received_time_stamp.toSec());
    last_inMsg_timestamp = finished_timestamp;
    }

}
void Scan_Analyser::time_sync_callback(const std_msgs::Float64 &time_sync_msg){
    received_time_stamp.fromSec(time_sync_msg.data);
//    time_diff.fromSec(ros::Time::now().toSec() - received_time_stamp.toSec());
//    ROS_INFO("received_time_stamp:%f",received_time_stamp.toSec());

}
std::vector<double> Scan_Analyser::interpolate_pose(double &time){
    std::vector<double> params;
    // todo use the std::lower_bound and std::lower_bound to find the nearst timestamp
    // todo map all possible value in the trajctory
    std::vector<double> time_stamps_mms;
    std::map<double,std::vector<double>>::iterator it;
    for (it = trajectories.begin(); it != trajectories.end(); ++it ){
        time_stamps_mms.push_back(it->first);
    }
    std::pair <double,double> time_pair = closest(time_stamps_mms, time);


//    long double time_before_transformed = MjdToUnix(GpsToMjd (0, 2035, time_pair.first));
//    long double time_after_transformed = MjdToUnix(GpsToMjd (0, 2035, time_pair.second));
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
void Scan_Analyser::read_MMS_trajectory(std::string path){//DONE confirmed right
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
            time_t t = gps2time(new gps_time_t {my_stod(*tmp.begin()),2035});
            trajectories[t] = params;
        }
    }
    std::cout<<"finished load trajectory"<<std::endl;
}

std::pair<double,double> Scan_Analyser::closest(std::vector<double> const& vec, double value) {
    auto const it_before = std::lower_bound(vec.begin(), vec.end(), value);
    auto const it_after = std::upper_bound(vec.begin(), vec.end(), value);
    if (it_before == vec.end() || it_after == vec.end() ) { return std::make_pair(-1.0,-1.0); }
//    if (DEBUGGING){
//        std::cout << "upper_bound at position " << (it_after - vec.begin()) << '\n';
//        std::cout << "lower_bound at position " << (it_before- vec.begin() - 1) << '\n';
//        std::cout<<"time_before found:"<<std::setprecision(15)<<*(it_before - 1)<<", ";
//        std::cout<<"time wanted:"<<std::setprecision(15)<<value<<", ";
//        std::cout<<"time_after found:" <<std::setprecision(15)<<*it_after<<std::endl;
//    }
    return std::make_pair(*(it_before - 1),*it_after);
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
//    tf::Matrix3x3 mat;
//    mat.setEulerZYX(param[3] * M_PI / 180,param[4]* M_PI / 180,param[5]* M_PI / 180);
//    transmatrix.setBasis(mat);
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
