#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>
#include "Scan_analyzer.h"
#include <list>
#include <limits>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "visualization_msgs/Marker.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
//#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>//convention of laserscan to pointcloud.
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Bool.h>
//#include <Eigen/Dense>
//Includes ICP
//pcd_io.h
//ply_io.h
//icp.h
//Dense
//opencv.hpp
//core.hpp
//mgproc.hpp
//highgui.hpp
//Inclludes Map to Rviz
//<visualization_msgs/Marker.h>
#include "AStar.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// Positionsupdate, benötigt für globales Mapping
// Wird von Control.cpp gepublisht

/////////////从odometry读出坐标和姿态

void Scan_analyzer::scannerCallback(const sensor_msgs::LaserScan &scan) {

	if (!init) {
		ROS_INFO("Waiting for Initialpose for 3sec");
		sleep(3);
		if (init)
			ROS_INFO("Got initial pose start scaning (%f.%f.%f)", robotPose.x,
					robotPose.y, robotPose.theta);
		else {
			ROS_INFO(
					"Warning: No initial pose start scanning with Pose (0.0,0.0,0.0)");
			scanPose.setIdentity();
		}
		init = true;
	} //if not init
	ROS_INFO("Getting new Scan...!");

//  1. transform Scan to PCL
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;
	current_cloud = laserscanToPCLCloud(scan);

//	2. get the transform Matrice from the current_cloud and last_cloud
	tf::Transform scan_transform;

	if (!last_cloud) {
		cloud_map_global = laserscanToPCLCloud(scan); //第一个循环将全局的cloud初始化
		Eigen::Affine3f transform_eigen = tfMatrixtoEigen(scanPose);
		pcl::transformPointCloud(*cloud_map_global, *cloud_map_global,
				transform_eigen);
		scan_transform = tf::Transform::getIdentity();
		ROS_INFO("last_cloud not initialized, cloud_map_global initialized");
		ROS_INFO("scan_transform initialized");
	} else {
		scan_transform = getScanOdom(last_cloud, current_cloud);
		//todo publish the delta-pose to odom_icp
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "/map";
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = scan_transform.getOrigin().getX();
		msg.pose.position.y = scan_transform.getOrigin().getY();
		tf::quaternionTFToMsg(scan_transform.getRotation(),
				msg.pose.orientation);
		icp_odom_pub.publish(msg);
		ROS_INFO("scan_transform updated");
	}
	ROS_INFO("OK");
// 3. update ScanPose
	update_scanpose(scan_transform);
//	4. get the transformed cloud, add it up to sum_cloud, pubilsh sum_cloud.

	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_global(
			new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Affine3f transform_eigen = tfMatrixtoEigen(scanPose);
	pcl::transformPointCloud(*current_cloud, *current_cloud_global,
			transform_eigen);

	*cloud_map_global += *current_cloud_global;

//	publish cloud_map_global;
	sensor_msgs::PointCloud2 ros_cloud;
	pcl::toROSMsg(*cloud_map_global, ros_cloud);
	ros_cloud.header.frame_id = "/map";
	cloud_pub.publish(ros_cloud);

//	5. use the transform matrix to update the coordinates of robot.
	last_cloud = current_cloud; //因为上一时刻和这一时刻两个点云近似，所以用这两个点云做ICP

	if(newpath){
		ProcessCloudToFindPath(current_cloud_global);
		newpath=false;
		ROS_INFO("##################################################### request newpath");
	}else{};

	if (count == 20) {
		Map_Raster = cv::Mat(size_x * 5, size_y * 5, CV_8UC3,
				cv::Scalar(0, 0, 0));
		ProcessCloudToFindPath(current_cloud_global);
		count = 0;
	} else {
		count++;
	}

}

Eigen::Affine3f Scan_analyzer::tfMatrixtoEigen(tf::Transform tf) {
	Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
	transform_eigen.translation() << tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ();
	std::cout << "Position now" << std::endl << transform_eigen.translation()
			<< std::endl;
	double roll, pitch, yaw;
	tf.getBasis().getRPY(roll, pitch, yaw);
	ROS_INFO("roll:%f,pitch:%f,yaw:%f", roll * 180 / M_PI, pitch * 180 / M_PI,
			yaw * 180 / M_PI);
	transform_eigen.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	std::cout << "current_cloud from local to global" << std::endl
			<< transform_eigen.matrix() << std::endl;
	return transform_eigen;
}
void Scan_analyzer::update_scanpose(tf::Transform transform) {

	tf::Vector3 temp = scanPose.getOrigin();
	if (std::isnan(temp.x()) || std::isnan(temp.y()) || std::isnan(temp.z()))
		scanPose = transform;		//第一次是transform,之后全是累乘
	//1.左乘还是右乘，将矩阵写出来，发现左乘（transform在左边）的话是旋转的机器人的坐标（x,y）在加上(delta_x,delta_y)不对
	//2.将4x4矩阵写出来后发现位置(X,Y)的变化是与旋转相关的，这样相乘以后会出现问题(delta_x,delta_y)会旋转后再加上（x,y）但是旋转的角度是机器人的姿势,这样相乘无意义。是错的
	//icp返回来的transform形式是

	/* Reminder: how transformation matrices work :

	 |-------> This column is the translation
	 | 1 0 0 x |  \
   | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	 | 0 0 1 z |  /
	 | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
	 */
	//所以将旋转处理。
	else {
		scanPose = scanPose * transform;

		//methode 1

//		tf::Matrix3x3 mat3_transform=transform.getBasis();
//		tf::Matrix3x3 mat3_scanPose=scanPose.getBasis();
//		tf::Matrix3x3 mat3_result=mat3_transform*mat3_scanPose;
//
//		tf::Vector3 translation_result = scanPose.getOrigin()+transform.getOrigin();
//		scanPose.setBasis(mat3_result);
//		scanPose.setOrigin(translation_result);

		//methode 2

//		double delta_x=transform.getOrigin().getX();
//		double delta_y=transform.getOrigin().getY();
//		double delta_angle=transform.getRotation().getAngle();
//		double x=scanPose.getOrigin().getX()+delta_x;
//		double y=scanPose.getOrigin().getY()+delta_y;
//		double angle=scanPose.getRotation().getAngle()+delta_angle;
//		scanPose.setOrigin(tf::Vector3 (x,y,0));
//		scanPose.setRotation(tf::createQuaternionFromYaw(angle));
	}

//	std::cout<<"scanPose"<<scanPose<<std::endl;
	double roll, pitch, yaw;
	tf::Vector3 print_origin = scanPose.getOrigin();
	tf::Matrix3x3(scanPose.getRotation()).getRPY(roll, pitch, yaw);
	ROS_INFO("The Current Pose is: (x, y, theta(deg)) (%f, %f, %f)",
			print_origin.x(), print_origin.y(), yaw * 180/M_PI);

	// publish Pose
	geometry_msgs::PoseStamped Pose;
	Pose.header.frame_id = "/map";
	Pose.pose.position.x = print_origin.getX();
	Pose.pose.position.y = print_origin.getY();

	Pose.pose.orientation.w = scanPose.getRotation().w();
	Pose.pose.orientation.x = scanPose.getRotation().x();
	Pose.pose.orientation.y = scanPose.getRotation().y();
	Pose.pose.orientation.z = scanPose.getRotation().z();

	pose_pub.publish(Pose);

}

//利用两次扫描的内容来更新机器人的位置，使用的是通过点云计算出来的柱子坐标，而不是点云的坐标。

Scan_analyzer::~Scan_analyzer() {
}

/*
 Konstruktor der Scan_analyser Klasse
 Es wird eine Roboterposition, verwendete Konstanten und die Publisher/Subscriber initialisiert
 */
Scan_analyzer::Scan_analyzer(ros::NodeHandle n) {

	count = 20;
	n_ = n;
	robotPose.x = 0.0;
	robotPose.y = 0.0;
	robotPose.theta = 0.0;

	rastersize = 5;		//unit cm 方格cell 的大小
	size_x = 120;		//size of map in x direction
	size_y = 120;		//size of map in y direction
	for (int i = 0; i < 120; i++) {
		for (int j = 0; j < 120; j++) {
			points_cloud_flag[i][j] = 0;
		}
	}
	Map_Raster_streched = cv::Mat(size_x * 5, size_y * 5, CV_8UC3,
			cv::Scalar(0, 0, 0));
	//# as null

	testposition_x = -2;
	testposition_y = -2;
	init = false; //makiert ob die Roboterposition initialisiert wurde
	//toDO Laborbearbeitung: Dies Folgenden Schwellwerte sind zu optimieren
	min_dist = 0.01;

	newpath=false;

	scanPose.setIdentity();
	scanPose.setOrigin( { -1.5, -1.5, 0 });	// initial Position of Robot

	target_final.x = 0.0;	//(1,1) used for debuging
	target_final.y = 0.0;

//	map_pole = n.advertise<visualization_msgs::Marker>("pole",1000);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_Scanner", 1);
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/map", 100);

	subScan = n.subscribe("scan", 1, &Scan_analyzer::scannerCallback, this);

	tracker_sub = n.subscribe("tracking/pose", 1,&Scan_analyzer::trackerCallback, this);	//todo adapt this subscribe
	cmdtonav_pub = n.advertise<geometry_msgs::PoseStamped>("/subTarget", 1);//to advertise it. only (x,y)
//	targettonav_sub=n.subscribe("/targetPos",1,&Scan_analyzer::locktarget,this);
	alligned_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/alligned_cloud", 1);
	icp_odom_pub = n.advertise<geometry_msgs::PoseStamped>("/icp_odom_difference", 1);
	require_new_path=n.subscribe("/plan_path",1, &Scan_analyzer::refreshpathCallback, this);

	if (!Map_Raster.empty()) {
		cv::imshow("Map", Map_Raster);
		cv::moveWindow("Map", 200, 200);
		cv::waitKey(1);
	}

}
void Scan_analyzer::refreshpathCallback(const std_msgs::Bool &request){

	if (request.data)
		newpath = true;
}
void Scan_analyzer::locktarget(const geometry_msgs::PoseStamped pose) {
	target_final.x = pose.pose.position.x;
	target_final.y = pose.pose.position.y;
}
// Umwandlung des Scans zu einer PCLPointCloud2
void Scan_analyzer::trackerCallback(const geometry_msgs::PoseStamped pose) {//todo check track to see is it's right
	robotPose.x = pose.pose.position.x;
	robotPose.y = pose.pose.position.y;
//	robotPose.theta=pose.pose.orientation.z;//todo use robotPose to initialize scanPose
	tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
			pose.pose.orientation.z, pose.pose.orientation.w);
//	std::cout<<"Orientation:"<<std::endl<<pose.pose.orientation.z<<std::endl;
	std::cout << "Position" << std::endl << pose.pose.position.x << ","
			<< pose.pose.position.y << std::endl;
	tf::Vector3 origin(robotPose.x, robotPose.y, 0);
	scanPose.setOrigin(origin);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robotPose.theta = yaw;
//	tf::Quaternion quat;
//	quat.setRPY(0,0,robotPose.theta);
	scanPose.setRotation(q);
	std::cout << "Orientation to scanPose-degree:" << yaw * 180 / M_PI
			<< std::endl;

	init = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Scan_analyzer::laserscanToPCLCloud(
		sensor_msgs::LaserScan scan) {
	laser_geometry::LaserProjection projector; // converting scan to ros::PointCloud or ros::PointCloud2
	sensor_msgs::PointCloud2 ros_pc2;
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr result(
			new pcl::PointCloud<pcl::PointXYZ>);

	// convert laserscan to PCL Cloud
	projector.projectLaser(scan, ros_pc2); // sensor_msgs::LaserScan to ros::PointCloud2
	pcl_conversions::toPCL(ros_pc2, pcl_pc2); // ROS to pcl::PCLPointCloud2
	pcl::fromPCLPointCloud2(pcl_pc2, *result); // pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>::Ptr

	return result;
}

tf::Transform Scan_analyzer::getScanOdom(
		pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud) {
	tf::Transform transform;	// Eine neune Transformation
	pcl::PointCloud<pcl::PointXYZ> current_cloud_transformed;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_transformed;
	try {//Fals ein Fehler Auftritt soll dieser Ignoriert werden und die leere Tranformation wird zurückgegeben
		Eigen::Matrix4f Tm;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(current_cloud);
		icp.setInputTarget(last_cloud);
		icp.setMaxCorrespondenceDistance(0.05);
		icp.setMaximumIterations(100);
//		icp.setTransformationEpsilon(1e-18); // this line renders icp disfunctional
//		icp.setRANSACOutlierRejectionThreshold(0.1);
//		icp.setEuclideanFitnessEpsilon(0.0001);
		icp.align(current_cloud_transformed);
//		*cloud_map_global+=current_cloud_transformed;
		Tm = icp.getFinalTransformation();
		std::cout << "has converged:" << icp.hasConverged() << " score: "
				<< icp.getFitnessScore() << std::endl;
//		if((!icp.hasConverged()) || icp.getFitnessScore()>0.01)
//			sleep(10);
		std::cout << "Scan_transform" << std::endl << Tm << std::endl;
//		pcl::PointCloud<pcl::PointXYZ>  Final;
//		icp.align(Final);
		Eigen::Affine3f transform_eigen = tfMatrixtoEigen(scanPose);
		pcl::transformPointCloud(current_cloud_transformed,
				current_cloud_transformed, transform_eigen);
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(current_cloud_transformed, cloud);
		cloud.header.frame_id = "/map";
		alligned_cloud_pub.publish(cloud);

		//Visualisieren Sie Ihre Ergebis mithilfe von openCV
		//Korekt zurücktransformierte Punkte mit grünen Pixeln
		//Fehler durch  schwarze (zweite Punktwolke)und rote (ICP Ergebnispunkte)

		//Geben Sie Ihre ICP Ergebnisse als Zahlenwerte auf der Konsole aus.
		//Eigen::Matrix4f Tm=icp.getFinalTransformation();
		//Füllen Sie zuerst den Translationsvektor nutzen Sie  static_cast<double> um Float zu Double zu wandeln
		tf::Vector3 origin;
		tf::Matrix3x3 mat3;
		origin.setValue(static_cast<double>(Tm(0, 3)),
				static_cast<double>(Tm(1, 3)), static_cast<double>(Tm(2, 3)));
		mat3.setValue(static_cast<double>(Tm(0, 0)),
				static_cast<double>(Tm(0, 1)), static_cast<double>(Tm(0, 2)),
				static_cast<double>(Tm(1, 0)), static_cast<double>(Tm(1, 1)),
				static_cast<double>(Tm(1, 2)), static_cast<double>(Tm(2, 0)),
				static_cast<double>(Tm(2, 1)), static_cast<double>(Tm(2, 2)));
		//Nutzen Sie die tf: Quaternion Klasse um die Rotationsmatrix in ein Quaternion zu wandeln getRotation()
		tf::Quaternion tfqt;
		mat3.getRotation(tfqt);
		transform.setOrigin(origin);
		transform.setRotation(tfqt);
		//Füllen Sie die Transformation
		//Zeigen Sie Ihre ICP openCV Matrix an scalieren und transformieren Sie Ihre Ergebnisse (= Punkt im Bild Zentrum) vorher.
	} catch (...) {

	}
	return transform;
}

std::pair<int, int> Scan_analyzer::getRasterCoordinate(double x, double y) {
	int cols = round((x * 100 + 300) / rastersize);		//转换为厘米
	int rows = round((y * 100 + 300) / rastersize);	//保证col和row都是正值，在x和y方向加3m.
	if (cols < 0 || cols > size_x) {
		ROS_INFO("X Coordinate false");
	}
	if (rows < 0 || rows > size_y) {
		ROS_INFO("Y Coordinate false");
	}
	return std::make_pair(cols, rows);
}

void Scan_analyzer::CloudMapToRasterMap(
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,
		AStar::Generator &generator) {
	std::vector<std::pair<int, int>> obscales_raster;
	for (int i = 0; i < current_cloud->points.size(); i++) {
		std::pair<int, int> coordinate;
		//y=ax+b (sx,sy)，(px,py)

		coordinate = getRasterCoordinate(current_cloud->points[i].x,
				current_cloud->points[i].y);
//		std::cout<<"Position of points"<<current_cloud->points[i].x<<","<<current_cloud->points[i].y<<std::endl;
//		std::cout<<"Position Raster Coordinate turn back"<<coordinate.first*rastersize/100<<","<<coordinate.second*rastersize/100<<std::endl;
		bool check = false;		//check= true , coordinate already exist.
		for (int j = -1; j < 2; j++) {
			for (int l = -1; l < 2; l++) {
				if(l==0||j==0) continue;
				if (points_cloud_flag[coordinate.first+j][coordinate.second+l] ){
				double d=	sqrt(pow(current_cloud->points[i].x-(coordinate.first+j)*rastersize/100,2)+pow(current_cloud->points[i].y-(coordinate.first+l)*rastersize/100,2));
				if(d<0.04){
					coordinate.first=coordinate.first+j;
					coordinate.second=coordinate.second+l;
					goto equ;
				}
				}
			}
		}
		equ:
		for (int j = 0; j < obscales_raster.size(); j++) {		//检查是否已经存在
			if (coordinate.first == obscales_raster[j].first
					&& coordinate.second == obscales_raster[j].second) {
				points_cloud_flag[coordinate.first][coordinate.second] = true;
				check = true;
				break;
			} else {
			}
		}
		if (!check) {
//		if(points_cloud_count[coordinate.first][coordinate.second]=true){
			obscales_raster.push_back(coordinate);
		}
	}
	//to set the global map
	for (int i = 0; i < 120; i++) {
		for (int j = 0; j < 120; j++) {
			if (points_cloud_flag[i][j] == true) {
				obscales_raster.push_back(std::make_pair(i,j));
				DrawUnitOnMap(std::make_pair(i, j), cv::Vec3i(255, 0, 0),
						Map_Raster);
			}
		}
	}

//	std::vector<std::pair<int,int>> collosion_filled=StrechMapOctree(obscales_raster);
	std::vector<std::pair<int, int>> collosion_filled = obscales_raster;
//	collosion_filled=StrechMapOctree(collosion_filled);
	for (int i = 0; i < collosion_filled.size(); i++) {
		generator.addCollision(
				{ collosion_filled[i].first, collosion_filled[i].second, 0 });
//		std::cout<<"Collision:"<<collosion[i].first<<","<<collosion[i].second<<std::endl;
	}
//	cv::Mat map (size_x*5,size_y*5,CV_8UC3,cv::Scalar(0,0,0));
//	map.zeros(size_x,size_y,)
	for (int i = 0; i < obscales_raster.size(); i++) {

		DrawUnitOnMap(obscales_raster[i], cv::Vec3i(255, 0, 0), Map_Raster);
//		DrawUnitOnMap(obscales_raster[i],cv::Vec3i (225,225,225),Map_Raster_streched);
		ROS_INFO("richtig_wall");
	}

//	cv::imshow("Map",rastermap);
//	cv::moveWindow("Map",0,200);
//	cv::waitKey(0.001);
}
std::vector<std::pair<int, int>> Scan_analyzer::StrechMapOctree(
		std::vector<std::pair<int, int>> collosion) {
	std::vector<std::pair<int, int>> collosion_fill;
	for (int i = 0; i < collosion.size(); i++) {
		collosion_fill.push_back(std::make_pair(collosion[i].first - 1, collosion[i].second - 1));
		collosion_fill.push_back(std::make_pair(collosion[i].first - 1, collosion[i].second));
		collosion_fill.push_back(std::make_pair(collosion[i].first - 1, collosion[i].second + 1));
		collosion_fill.push_back(std::make_pair(collosion[i].first, collosion[i].second - 1));
		collosion_fill.push_back(std::make_pair(collosion[i].first, collosion[i].second));
		collosion_fill.push_back(std::make_pair(collosion[i].first, collosion[i].second + 1));
		collosion_fill.push_back(std::make_pair(collosion[i].first + 1, collosion[i].second - 1));
		collosion_fill.push_back(std::make_pair(collosion[i].first + 1, collosion[i].second));
		collosion_fill.push_back(std::make_pair(collosion[i].first + 1, collosion[i].second + 1));
	}
	std::set<std::pair<int, int>> s;
	unsigned size = collosion_fill.size();
	for (unsigned i = 0; i < size; ++i)
		s.insert(collosion_fill[i]);
	collosion_fill.assign(s.begin(), s.end());
	return collosion_fill;
}

std::pair<int, int> Scan_analyzer::cvtRasterMap(
		geometry_msgs::PoseStamped target) {
	double x_target = target.pose.position.x;
	double y_target = target.pose.position.y;
//	std::cout<<"Target raster coordinate"<<x_target<<","<<y_target<<std::endl;
	return getRasterCoordinate(x_target, y_target);

}

//void Scan_analyzer::getStepList(std::vector<std::pair<int,int>> Raster_Path_List){
//	std::vector<std::pair<double,double>> temp_target;
//	for(int i=0;i< Raster_Path_List.size();i++){
//		double temp_x,temp_y;
//		temp_x=Raster_Path_List[i].first*rastersize;
//		temp_y=Raster_Path_List[i].second*rastersize;
//		temp_target.push_back(std::make_pair(temp_x,temp_y));
//	}
//}
std::vector<std::pair<int, int>> Scan_analyzer::AStar_PathToTarget(
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,
		std::pair<int, int> target, std::pair<int, int> pose_now) {
	AStar::Generator generator;
	generator.setWorldSize( { size_x, size_y });		//Important
	generator.setHeuristic(AStar::Heuristic::euclidean);
	generator.setDiagonalMovement(false);
	CloudMapToRasterMap(current_cloud, generator);
	std::cout << "Generate path ... \n";
	std::vector<AStar::Vec2i> pathtotarget;
	pathtotarget = generator.findPath( { pose_now.first, pose_now.second, 0 }, {
			target.first, target.second, 0 });
	std::vector<std::pair<int, int>> path_XY;
	for (int i = 0; i < pathtotarget.size(); i++) {
		path_XY.push_back(std::make_pair(pathtotarget[i].x, pathtotarget[i].y));
	}
	return path_XY;
}
void Scan_analyzer::ProcessCloudToFindPath(
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud) {
	//1. process the position of target and robot to find the correspond position in the rastermap
	//2. output the map in the OpenCV window
	//3. convert the raster coordinate of path to normal global coordinate of control and publish it.
	geometry_msgs::PoseStamped target;

	target.pose.position.x = target_final.x;
	target.pose.position.y = target_final.y;	 //todo manually set;
	std::cout << "target" << target_final.x << "," << target_final.y
			<< std::endl;
	std::pair<int, int> target_raster = cvtRasterMap(target);//todo replace with right one//todo initial to be the origin
	std::cout << "target raster coordinate" << target_raster.first << ","
			<< target_raster.second << std::endl;
	/////target in rastermap
	geometry_msgs::PoseStamped pose_now;
	pose_now.pose.position.x = scanPose.getOrigin().getX();	//todo scanPose must be updated by camera ->Trackercallback
	pose_now.pose.position.y = scanPose.getOrigin().getY();
	std::cout << "robotpose" << pose_now.pose.position.x << ","
			<< pose_now.pose.position.y << std::endl;
	std::pair<int, int> pose_raster = cvtRasterMap(pose_now);
	std::cout << "Pose raster coordinate" << pose_raster.first << ","
			<< pose_raster.second << std::endl;
	/////pose in rastermap
// 	cv::Mat Map_Raster;
	DrawUnitOnMap(target_raster, cv::Vec3i(0, 0, 255), Map_Raster);
	ROS_INFO("richtig_target");
	DrawUnitOnMap(pose_raster, cv::Vec3i(0, 255, 0), Map_Raster);
	ROS_INFO("richtig_pose");

	std::vector<std::pair<int, int>> path = AStar_PathToTarget(current_cloud,
			target_raster, pose_raster);

	for (int i = 1; i < path.size() - 1; i++) {
		DrawUnitOnMap(path[i], cv::Vec3i(255, 255, 0), Map_Raster);
		ROS_INFO("richtig_path");
	}
	//get the step list and then pubish it.
	std::vector<Scan_analyzer::Target> steplist;
	for (int i = 0; i < path.size(); i++) {
		Scan_analyzer::Target target;
		target.x = (path[i].first * rastersize - 300) / 100;//Important::300cm off set
		target.y = (path[i].second * rastersize - 300) / 100;
		steplist.push_back(target);
// 		std::cout<<"path to be published:("<<target.x<<","<<target.y<<std::endl;
	}
	int nSize = (int) steplist.size();
	for (int i = nSize - 3; i >= nSize - 3; --i) {
		geometry_msgs::PoseStamped coor_msg;
		coor_msg.header.frame_id = "/map";
		coor_msg.pose.position.x = steplist[i].x;
		coor_msg.pose.position.y = steplist[i].y;

// 	 	DrawUnitOnMap(path[i],cv::Vec3i (255,0,255),Map_Raster);
		std::cout << coor_msg.pose.position.x << coor_msg.pose.position.y
				<< std::endl;

		cmdtonav_pub.publish(coor_msg);
	}
	cv::imshow("Map", Map_Raster);
	cv::moveWindow("Map", 200, 200);
	cv::waitKey(100);
}
/*
 Main
 */
void Scan_analyzer::DrawUnitOnMap(std::pair<int, int> position, cv::Vec3i color,
		cv::Mat &mapp) {
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			mapp.at<cv::Vec3b>(position.first * 5 + i, position.second * 5 + j)[1] = color[0]; //Rot- Robot
			mapp.at<cv::Vec3b>(position.first * 5 + i, position.second * 5 + j)[2] = color[1];
			mapp.at<cv::Vec3b>(position.first * 5 + i, position.second * 5 + j)[3] = color[2];
//			std::cout<<"Color"<<color[1]<<","<<color[2]<<","<<color[3]<<","<<std::endl;
		}
	}
//	 return mapp;
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "Scan_analyzer");
	ros::NodeHandle n;
	Scan_analyzer m = Scan_analyzer(n);
	ros::spin();
	return 0;
}
