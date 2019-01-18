#include "scan_analyser.h"
#include "nav_msgs/Odometry.h"
#include "velodyne_pointcloud/pointcloudXYZIR.h"
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud2.h"
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
Scan_Analyser::Scan_Analyser(ros::NodeHandle nh)
{   double start_time = 230089.056408;
    ROS_INFO("ROS_TIME now %f",ros::Time::now().toSec());
    time_diff.fromSec(ros::Time::now().toSec() - start_time);
    std::string path = "/home/jie/catkin_ws/data/Trajectory_CSV.txt";
    read_MMS_trajectory( path );

    std::string path_ = "/home/jie/catkin_ws/data/points_left.ply";

    pcl::io::loadPLYFile(path_,pcl);

//    ROS_INFO("%f",time_test);
//    std::cout<<"search for time:"<<std::setprecision(20)<<time_test<<std::endl;


    time_stamp_sub= nh.subscribe("/trigger_timestamps", 1, &Scan_Analyser::time_sync_callback,this);
    velodyne_scan_sub = nh.subscribe("/velodyne_packets",1,&Scan_Analyser::scanner_callback,this);
    transformed_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/velo_transformed",1);
    while (ros::ok()){
        ros::Duration(0.5).sleep();
        // todo return a time to find the pose, the transform the pose using the tf
        double transformed_time = ros::Time::now().toSec()-time_diff.toSec();
        ROS_INFO("Time difference %f",time_diff.toSec());
        ROS_INFO("Time transformed %f",transformed_time);
//        double time_test = 230148.877219;
        std::vector<double> params = interpolate_pose(transformed_time);
        tf::Transform T = generate_tf_matrix(params);
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(pcl,output_msg);

        sensor_msgs::PointCloud2 output = transform_cloud(T,output_msg);
        output.header.frame_id = "velodyne";

        transformed_pc_pub.publish(output);
        ROS_INFO("Hello world!");
    }
}
Scan_Analyser::~Scan_Analyser(){

}
// TODO interpolate the scanpose
void Scan_Analyser::scanner_callback(const velodyne_msgs::VelodyneScan &scan_msg){
    nav_msgs::Odometry odom;
    //convert_3d_lidar(scan_msg,odom);
}
void Scan_Analyser::time_sync_callback(const std_msgs::Float64 &time_sync_msg){
    received_time_stamp.fromSec(time_sync_msg.data);
    ROS_INFO("received_time_stamp:%f",received_time_stamp.toSec());
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
    std::vector<double> param_before = trajectories[time_pair.first];
    std::vector<double> param_after= trajectories[time_pair.second];

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
             //   std::cout<<*param_it<<std::endl;
            }

//            ROS_INFO("%s",tmp.at(0).c_str());
            trajectories[my_stod(*tmp.begin())] = params;
//            ROS_INFO("%f",my_stod(*tmp.begin()));

            if (DEBUGGING){
//                ROS_INFO("********************************************");
//                ROS_INFO("params: %s",tmp.at(0).c_str());
//                ROS_INFO("params: %f",params.at(0));
//                ROS_INFO("params: %f",params.at(1));
//                ROS_INFO("params: %f",params.at(2));
//                ROS_INFO("params: %f",params.at(3));
//                ROS_INFO("params: %f",params.at(4));
//                ROS_INFO("params: %f",params.at(5));
//                std::cout<<std::setprecision(15)<<params.at(0)<<std::endl;
//                std::cout<<std::setprecision(15)<<params.at(1)<<std::endl;
//                std::cout<<std::setprecision(15)<<params.at(2)<<std::endl;
//                std::cout<<std::setprecision(15)<<params.at(3)<<std::endl;
//                std::cout<<std::setprecision(15)<<params.at(4)<<std::endl;
            }
        }
    }

}
std::pair<double,double> Scan_Analyser::closest(std::vector<double> const& vec, double value) {
    auto const it_before = std::lower_bound(vec.begin(), vec.end(), value);
    auto const it_after = std::upper_bound(vec.begin(), vec.end(), value);

    if (it_before == vec.end() || it_after == vec.end() ) { return std::make_pair(-1.0,-1.0); }
    std::cout << "upper_bound at position " << (it_after - vec.begin()) << '\n';
    std::cout << "lower_bound at position " << (it_before- vec.begin() - 1) << '\n';
    std::cout<<"time_before found:"<<std::setprecision(15)<<*(it_before - 1)<<",    ";
    std::cout<<"time wanted:"<<std::setprecision(15)<<value<<",";
    std::cout<<"time_after found:" <<std::setprecision(15)<<*it_after<<std::endl;
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
    tf::Quaternion quat = tf::Quaternion::getIdentity();
    quat.setRPY(param[3],param[4],param[5]);
    transmatrix.setRotation(quat);
    transmatrix.setOrigin(tf::Vector3(param[0],param[1],param[2]));
    return transmatrix;
}
sensor_msgs::PointCloud2 Scan_Analyser::transform_cloud(const tf::Transform T, const sensor_msgs::PointCloud2 &msg){
    //TODO transform the cloud using T
    sensor_msgs::PointCloud2 transformed_pc;
    pcl_ros::transformPointCloud("velo", T, msg, transformed_pc);
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
