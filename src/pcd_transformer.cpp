#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <pcd_viewer/RosDynamicReconfigureConfig.h>


ros::Publisher pub;
sensor_msgs::PointCloud2 output_cloud;
std::string pcd_name;

float degree_in_rad; 


void callback(pcd_viewer::RosDynamicReconfigureConfig &config, uint32_t level) {
  degree_in_rad = config.degree_slider * M_PI/180;
  ROS_INFO_STREAM(
              "rotation degree: " << config.degree_slider << std::endl
          );
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);


  // transform pcl work!
  tf::Quaternion q;
  q.setRPY(0, degree_in_rad, 0);
  // tf::Matrix3x3 R(q);

  tf::Vector3 translation(0.0, 0.0, 0.0);
  tf::Transform transform(q, translation);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    tf::Vector3 point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    tf::Vector3 transformed_point = transform * point;
    cloud->points[i].x = transformed_point.x();
    cloud->points[i].y = transformed_point.y();
    cloud->points[i].z = transformed_point.z();
  }

  pcl::toROSMsg(*cloud, output_cloud);
  output_cloud.header.frame_id = "map"; // Replace with your desired frame ID
  output_cloud.header.stamp = ros::Time::now();

  pub.publish(output_cloud);
}


bool savePclService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    pcl::fromROSMsg(output_cloud, pcl_cloud);
    // Define the filename and save the point cloud to a .pcd file
    std::string package_path = ros::package::getPath("pcd_viewer");
    std::string filename = package_path + "/pcd/" + pcd_name +".pcd";

    if (pcl::io::savePCDFileASCII(filename, pcl_cloud) == -1)
    {
        ROS_ERROR("Failed to save PCD file!");
        return false;
    }
    ROS_ERROR("PCL to PCD Completed!");

    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_transform");
  ros::NodeHandle nh("~");

  nh.getParam("pcd_name", pcd_name);

  ros::ServiceServer service = nh.advertiseService("save_pcl", savePclService);

  dynamic_reconfigure::Server<pcd_viewer::RosDynamicReconfigureConfig> server;
  dynamic_reconfigure::Server<pcd_viewer::RosDynamicReconfigureConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Subscriber sub = nh.subscribe("/pcd_opener/point_cloud", 1, cloudCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_rotated", 1);
  

  // Spin the node
  ros::spin();

  return 0;
}
