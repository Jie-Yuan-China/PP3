# PP3
PP3.3 Time sync using 2 velodyne laserscanners using ntp server by GPS receiver

# Usage
1. roscore

2. for 64E:
rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet _calibration:=/home/jie/catkin_ws/src/velodyne/velodyne_pointcloud/params/64e_utexas.yaml 

for VLP16:
rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet _calibration:=/home/jie/catkin_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml 
3. rosrun sync_velodyne_pcl sync_velodyne_pcl_node 
3. rosbag play 
4. rviz use frame velodyne
## hints
for pcap
	1. make sure Note 1 is done.
 rosrun velodyne_driver velodyne_node _model:=VLP16 _pcap:=/home/jie/23012019-14.pcap




#Note: 
Changed the line 290 of input.cc of velodyne_driver to get the real timestamp from pcap


-            pkt->stamp = ros::Time::now();
+            pkt->stamp = ros::Time(header->ts.tv_sec, header->ts.tv_usec * 1000);

Changed line 17-18 of pointcloudXYZIR.cc to add the info of azimuth, which used for correction of points

+     point.azimuth = azimuth;
+     point.distance = distance;


Changed line 32-34 of point_types.h to add definition of azimuth, distance, and timestamp

+    uint16_t azimuth;                   ///< 0-35999>, divide by 100 to get degrees
+    float distance;
+    time_t timestamp;                   ///< timestamp> used for syncronization

# Magic Notes

It's not necessary to define a new message type, just add some new attributes to the pcl::point class then it will be directly forwarded, with no need to change the message size
