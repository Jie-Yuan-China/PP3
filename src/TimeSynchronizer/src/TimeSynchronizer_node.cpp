#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>
//#include <std_msgs/Float64.h>
#include <message_filters/sync_policies/approximate_time.h>
//TODO : because std_msgs::Float64 has no timestamp in header, Synchronizer won't work here.
typedef message_filters::sync_policies::ApproximateTime<std_msgs::Float64,velodyne_msgs::VelodyneScan> NoCloudSyncPolicy;
typedef message_filters::Subscriber<velodyne_msgs::VelodyneScan> scan_sub;
typedef message_filters::Subscriber<std_msgs::Float64> time_sub;
class MyClass {
//message_filters::Subscriber<std_msgs::Float64> * timestamp_pi_sub_;
//message_filters::Subscriber<std_msgs::Float64> * timestamp_VLP16_sub_;
message_filters::Subscriber<velodyne_msgs::VelodyneScan> *laserscan_64E_sub_;
//message_filters::Subscriber<velodyne_msgs::VelodyneScan> *laserscan_VLP16_sub_;
message_filters::Subscriber<std_msgs::Float64> * time_sub_;
message_filters::Synchronizer<NoCloudSyncPolicy>* mysync;

// method definitions needed here
public:
    ros::NodeHandle n;
public:
    MyClass(const ros::NodeHandle nh);
    void callbackMethod (const velodyne_msgs::VelodyneScan::ConstPtr& VLP16_msg,
                         const std_msgs::Float64::ConstPtr& E64_msg);
    ~MyClass();
};

MyClass::MyClass(const ros::NodeHandle nh)
{
//std::string time_stamp_tpc("/trigger_timestamp");
// TODO : change the Name of topics.
    n = nh;
std::string VLP16_Scan_tpc("/velodyne_points");
std::string E64_Scan_tpc("/velodyne_points");
std::string time_stamp_tpc("/trigger_timestamp");
int q = 5; //queue size


time_sub_ = new message_filters::Subscriber<std_msgs::Float64>(n, time_stamp_tpc, q);
//laserscan_VLP16_sub_ = new pcl_sub(n, VLP16_Scan_tpc, q);
laserscan_64E_sub_ = new message_filters::Subscriber<velodyne_msgs::VelodyneScan>(n, E64_Scan_tpc, q);

mysync = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *time_sub_, *laserscan_64E_sub_);

mysync->registerCallback(boost::bind(&MyClass::callbackMethod, this, _1, _2));
}
MyClass::~MyClass(){}
//The callback method
void MyClass::callbackMethod (const velodyne_msgs::VelodyneScan::ConstPtr& VLP16_msg,
                              const std_msgs::Float64::ConstPtr& E64_msg)
{
    ROS_INFO("Hello world!");
     //Your processing code
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "TimeSynchronizer");
  ros::NodeHandle nh;
  MyClass  ma (nh);
//  message_filters::Subscriber<velodyne_msgs::VelodyneScan> scan_sub(nh, "/velodyne_packets", 1);
//  message_filters::Subscriber<std_msgs::Float64> time_sub(nh, "/trigger_timestamp", 1);
//  typedef message_filters::sync_policies::ApproximateTime<velodyne_msgs::VelodyneScan, std_msgs::Float64> MySyncPolicy;
//  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), scan_sub, time_sub);
//  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
}
