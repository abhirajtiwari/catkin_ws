#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
ros::Publisher pub;

void apply_filters(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {   
    //Create containers to hold the message and output
    pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    //Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    //remove statistical outliers
    /*pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(cloud_filtered);
*/
    //Voxel Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(cloudPtr);
    float leaf_size = 0.1;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(cloud_filtered);
    
    //Convert to ROS datatype again
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_filter");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcl", 100);
    ros::Subscriber pcl_sub = nh.subscribe("rs_pcl", 100, apply_filters);
    ros::spin();
    return 0;
}
