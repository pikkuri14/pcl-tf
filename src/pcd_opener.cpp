#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "pcd_node");
    ros::NodeHandle nh("~");

    std::string pcd_dir;
    nh.getParam("pcd_dir", pcd_dir);

    
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    std::string pcd_filename = pcd_dir;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s.", pcd_filename.c_str());
        return -1;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        pub.publish(cloud_msg);
        loop_rate.sleep();
    }

    return 0;
}

