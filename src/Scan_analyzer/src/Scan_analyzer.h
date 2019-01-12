#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "tutorial/Position_with_Header.h"
//#include "laserscanner/Cirlce.h"
//#include <tuple>
#include <list>
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include "AStar.hpp"
#include <std_msgs/Bool.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Scan_analyzer {

public:
	//datatypes
	struct pole {
		float x, y, r;
	};
	struct Target{
		float x,y;
	};
	// Attributes
	tf::Transform scanPose;
    Target target_final;
	ros::NodeHandle n_;
	float min_dist;
	bool init;
	float rastersize;		//unit cm 方格cell 的大小
	int size_x;//size of map in x direction
	int size_y;//size of map in y direction

	float testposition_x;
	float testposition_y;

	int count;
	bool points_cloud_flag[120][120];
	bool newpath;
	geometry_msgs::Pose2D robotPose;
	ros::Publisher pub;
	ros::Publisher map_pole;
	ros::Subscriber subScan;
	ros::Publisher marker_pub;
	ros::Subscriber subRobPos;
	ros::Subscriber subRobInitPos;
	pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_global;
	ros::Publisher pose_pub;
	ros::Publisher cloud_pub;
	ros::Publisher pub_last_cloud;
	ros::Publisher pub_current_cloud;
	ros::Subscriber tracker_sub;
	ros::Publisher cmdtonav_pub;
	ros::Subscriber targettonav_sub;
	ros::Publisher alligned_cloud_pub;
	ros::Publisher icp_odom_pub;
	ros::Subscriber require_new_path;

	cv::Mat Map_Raster;
	cv::Mat Map_Raster_streched;
	// Methods
	Scan_analyzer(ros::NodeHandle n);
	~Scan_analyzer();
	void scannerCallback(const sensor_msgs::LaserScan &scan);
	tf::Transform getScanOdom(pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserscanToPCLCloud(sensor_msgs::LaserScan scan);
	void update_scanpose(tf::Transform transform);
//	std::vector<std::pair<int, int>> AStar_PathToTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud);
	void CloudMapToRasterMap(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,	AStar::Generator &generator);
	void trackerCallback(const geometry_msgs::PoseStamped pose);
	std::pair<int,int> getRasterCoordinate(double x,double y);
	std::pair<int,int> cvtRasterMap(geometry_msgs::PoseStamped target);
	std::pair<int,int> AdaptPoseToRasterMap(geometry_msgs::PoseStamped pose);
	void getStepList(std::vector<std::pair<int,int>> Raster_Path_List);
	std::vector<std::pair<int,int>> AStar_PathToTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,std::pair<int,int> target,std::pair<int,int> pose_now);
	void ProcessCloudToFindPath(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud);
	void locktarget(const geometry_msgs::PoseStamped pose);
	cv::Mat getCVMap(std::vector<std::pair<int,int>> collosion);
	void drawTargetandPosetoMap(std::pair<int,int> pose_raster,std::pair<int,int> target_raster,cv::Mat &Map_Raster,std::vector<std::pair<int,int>> path);
	std::vector<std::pair<int,int>> StrechMapOctree(std::vector<std::pair<int,int>> collosion);
	void DrawUnitOnMap(std::pair<int,int> position,cv::Vec3i color,cv::Mat &mapp);
	Eigen::Affine3f tfMatrixtoEigen(tf::Transform tf);
	void refreshpathCallback(const std_msgs::Bool &request);
};
