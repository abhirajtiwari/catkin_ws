#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>

#define MAP_WIDTH 61 //m
#define MAP_HEIGHT 61 //m
#define MAP_RESOLUTION 0.5 //m/cell
#define COST_PER_POINT 5

ros::Publisher turn_cmd;


void countObstacles (const sensor_msgs::PointCloud2::ConstPtr& icloud) {

	std_msgs::String msg;

	//Convert Point Cloud so that it is useable 
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(*icloud, *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>); //cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*input_cloud);	

	//make z = 0
    int p_count=0,n_count=0,it_count=0;
	for (auto p : input_cloud->points) {
	if(p.x<0)
		n_count++;
	else 
		p_count++;
	it_count++;
	}
	if (it_count==0)
		{std::cout<<"straight";
			msg.data="straight";}
	else if (n_count>p_count)
		{std::cout<<"right\n";
			msg.data="right";}
	else 
		{std::cout<<"left\n";
			msg.data="left";}
	
	turn_cmd.publish(msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rs_override");
	ros::NodeHandle nh;
	turn_cmd= nh.advertise<std_msgs::String>("override", 10);
	/* ros::Subscriber pl_sub = nh.subscribe("plane", 10, addPlane); */
	ros::Subscriber ob_sub = nh.subscribe("obstacle", 10, countObstacles);
	/* ros::Subscriber pose_sub = nh.subscribe("odom", 50, getPose); */
	ros::spin();
	return 0;
}

