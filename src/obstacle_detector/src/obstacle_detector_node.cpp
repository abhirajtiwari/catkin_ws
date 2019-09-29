#include <vector>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>

ros::Publisher pcl_pub;
ros::Publisher plane_pub;

double distance_threshold = 0.1;

void detect_obstacle(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
	/* bool done = false; */
	/* while (!done) { */
	try {
		pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2 obstacles;

		//Convert to PCL data type
		pcl_conversions::toPCL(*cloud_msg, *cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(*cloud,*temp_cloud);	

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PCDWriter writer;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (distance_threshold);
		
		int i=0, nr_points = (int) temp_cloud->points.size ();
		// Segment the largest planar component from the remaining cloud
		if (nr_points < 3) throw -1;
		seg.setInputCloud (temp_cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			// break;
		}

		//Angle calculation
		double a=coefficients->values[0], b=coefficients->values[1], c=coefficients->values[2];
		double s = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
		std::vector<double> normal = {a/s, b/s, c/s};
		std::vector<double> xz = {0,-1,0};
		double dot = std::inner_product(std::begin(normal), std::end(normal), std::begin(xz), 0.0);
		double angle = acos(dot);
		std::cout << angle*(180/M_PI) << std::endl;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (temp_cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		// std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*temp_cloud = *cloud_f;

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (temp_cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.2); // 2cm
		ec.setMinClusterSize (20);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (temp_cloud);
		ec.extract (cluster_indices);
		pcl::PointCloud<pcl::PointXYZ> full_frame;

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (temp_cloud->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			// std::stringstream ss;
			// ss << "cloud_cluster_" << j << ".pcd";
			// writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
			if(j==0)
				full_frame=*cloud_cluster;	
			else full_frame+=*cloud_cluster;	
			j++;
		}

		// return full_frame.makeShared();
		full_frame.header.frame_id="pcl_frame";
		pcl_pub.publish(full_frame);
		cloud_plane->header.frame_id="pcl_frame";

		plane_pub.publish(*cloud_plane);
		/* done = true; */
	}
	catch (...) {
		std::cout << "Error" << std::endl;
		ROS_DEBUG("No points for plane segmentation");
	}
	/* } */
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "obstacle_det");
	ros::NodeHandle nh;
	
	typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
    pcl_pub = nh.advertise<PCLCloud>("obstacle", 100);
	plane_pub = nh.advertise<PCLCloud>("plane", 100);
	ros::Subscriber pcl_sub = nh.subscribe("filtered_pcl", 100, detect_obstacle);
	ros::spin();
	return 0;
}
