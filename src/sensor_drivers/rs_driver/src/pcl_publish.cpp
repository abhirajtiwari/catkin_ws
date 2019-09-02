#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// class filter_options
// {
// public:
//     filter_options(const std::string name, rs2::filter &filter);
//     filter_options(filter_options &&other);
//     std::string filter_name;     //Friendly name of the filter
//     rs2::filter &filter;         //The filter in use
//     std::atomic_bool is_enabled; //A boolean controlled by the user that determines whether to apply the filter or not
// };

pcl_ptr points_to_pcl(const rs2::points &points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    std::cout << "Start" << std::endl;
    for (auto &p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    std::cout << "End" << std::endl;

    return cloud;
}

int main(int argc, char *argv[]) try
{
    ros::init(argc, argv, "rs_all_stream", ros::init_options::AnonymousName);

    typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
    ros::NodeHandle n;
    ros::Publisher pcl_pub = n.advertise<PCLCloud>("rs_pcl", 100);
    ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("rs_rgb", 100);

    // ros::Rate loop_rate(30);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
    // Spatial    - edge-preserving spatial smoothing
    rs2::spatial_filter spat_filter;
    rs2::threshold_filter thresh_filter;
    rs2::decimation_filter dec_filter;

    // Initialize a vector that holds filters and their options
    // std::vector<filter_options> filters;
    // filters.emplace_back("Spatial", spat_filter);

    while (ros::ok())
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();
        if (!depth) // Should not happen but if the pipeline is configured differently
            break;

        rs2::frame filtered = depth; // Does not copy the frame, only adds a reference
        
        // filtered = spat_filter.process(filtered);
        thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4.0);
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
        filtered = dec_filter.process(filtered);
        filtered = thresh_filter.process(filtered);
        
        //RGB frame
        auto colour = frames.get_color_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(filtered);

        auto pcl_points = points_to_pcl(points);
        // pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(pcl_points);
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(0.0, 5.0);

        // // Create the filtering object
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud (pcl_points);
        // sor.setMeanK (50);
        // sor.setStddevMulThresh (1.0);
	

        auto pointer_u = *pcl_points;
//      pass.filter(pointer_u);
	    //sor.filter (pointer_u);
        pointer_u.header.frame_id = "pcl_frame";
        //Publish PCL after converting
        pcl_pub.publish(pointer_u);

        //RS2 to matrix
        cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)colour.get_data(), cv::Mat::AUTO_STEP);
        cv_bridge::CvImage lo_img;

        lo_img.encoding = "rgb8";               // or which enconding your data has
        lo_img.header.stamp = ros::Time::now(); //  or whatever timestamp suits here;
        lo_img.header.frame_id = "rgb_frame";   // frame id as neededby you
        lo_img.image = color;                   // point cv_bridge to your object

        rgb_pub.publish(lo_img.toImageMsg());
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
