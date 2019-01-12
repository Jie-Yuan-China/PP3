#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/PointCloud2.h"

// todo add the function to sync VLP16 and 64E
class Sync
{
public:
    double time_diff;
    ros::Publisher pub_time;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_time;
    ros::NodeHandle n_;
public:
    void PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void UTCCallback(const std_msgs::Float64::ConstPtr& msg);
    void Sync_time(const float &last_time_signal, ros::Time &last_ros_time );
    Sync(ros::NodeHandle nh);
    ~Sync();
};
Sync::~Sync(){

}
Sync::Sync(const ros::NodeHandle nh){
    n_ = nh;
    time_diff = 0.0;
    pub_time = n_.advertise<std_msgs::Float64>("sychronized_time",1);
    sub_pcl = n_.subscribe("velodyne_points",1, &Sync::PointCloud2Callback,this);
    sub_time = n_.subscribe("trigger_timestamps",1,&Sync::UTCCallback,this);
}
void Sync::PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("I received pointcloud.");
    ROS_INFO ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    double time = ros::Time::now().toSec();
    ROS_INFO("ROS TIME NOW: %f", time);
    double utc_time = time + time_diff;
    ROS_INFO("The time after Sync is %f", utc_time);
    std_msgs::Float64 time_sync;
    time_sync.data = utc_time;
    pub_time.publish(time_sync);
    // convert the ros time to UTC ?
}
void Sync::UTCCallback(const std_msgs::Float64::ConstPtr& msg){
    float_t sync_time = msg->data;
    time_diff = sync_time - ros::Time::now().toSec();
    ROS_INFO("I heared the time message: % f", sync_time);
    ROS_INFO("I got time difference %f", time_diff);
}
void Sync::Sync_time(const float &last_time_signal, ros::Time &last_ros_time ){
    //todo save the time now then, cal the difference between last_ros_time and the real ros time,
    //then add the difference to every pointcloud until the next sync_time comes.
    //so that every timestamp of pointcloud receives the real time
    ros::Time ros_time_now =  ros::Time::now();
    ros::Duration time_diff = ros_time_now - last_ros_time;


}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_velodyne_pcl_node");
    ros::NodeHandle nh;
    Sync syhcr = Sync(nh);
//    ros::Publisher pub_time = nh.advertise<std_msgs::Float64>("sychronized_time",1);
//    ros::Publisher pub_time = n.advertise<std_msgs::String>("chatter", 1000);
    ros::spin();

    return 0;
}
