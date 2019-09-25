#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>

#define MAP_WIDTH 30
#define MAP_HEIGHT 30
#define MAP_RESOLUTION 0.5

ros::Publisher occu_grid_pub;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose fake_pose;

void getPose (const nav_msgs::Odometry::ConstPtr& data) {
	current_pose = data->pose.pose;
}

void addObstacles (const sensor_msgs::PointCloud2::ConstPtr& icloud) {

	//temporary
	fake_pose.position.x = 0;
	fake_pose.position.y = 0;
	fake_pose.position.z = 0;
	fake_pose.orientation.x = 0;
	fake_pose.orientation.y = 0;
	fake_pose.orientation.z = 0;
	fake_pose.orientation.w = 0;

	nav_msgs::OccupancyGrid grid;
	grid.header.frame_id = "temp_map";
	grid.header.stamp = ros::Time::now();
	grid.info.resolution = MAP_RESOLUTION;
	grid.info.width = MAP_WIDTH;
	grid.info.height = MAP_HEIGHT;
	/* grid.info.origin = current_pose; */
	grid.info.origin = fake_pose;

	//Convert Point Cloud so that it is useable CONTINUE FROM HERE
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(*icloud, *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>); //cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*input_cloud);	

	//make z = 0
	for (auto &p : input_cloud->points) p.z = 0; 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "costmap_generator");
	ros::NodeHandle nh;
	occu_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("ocgrid", 10);
	ros::Subscriber ob_sub = nh.subscribe("obstacle", 10, addObstacles);
	/* ros::Subscriber pose_sub = nh.subscribe("odom", 50, getPose); */
	ros::spin();
	return 0;
}

