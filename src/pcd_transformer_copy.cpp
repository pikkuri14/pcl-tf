#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <dynamic_reconfigure/server.h>
#include <pcd_viewer/RosDynamicReconfigureConfig.h>

float degree_in_rad;

void callback(pcd_viewer::RosDynamicReconfigureConfig &config, uint32_t level) {
    degree_in_rad = config.degree_slider * M_PI / 180;
    ROS_INFO_STREAM("Slider value: " << config.degree_slider << std::endl);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    dynamic_reconfigure::Server<pcd_viewer::RosDynamicReconfigureConfig> server;
    dynamic_reconfigure::Server<pcd_viewer::RosDynamicReconfigureConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate rate(10.0);

    while (ros::ok()) {
        // Update transform based on the current degree_in_rad value
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "rotated_cloud_frame";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, degree_in_rad, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transformStamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transformStamped);

            
        rate.sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("Sending!");
    }

    return 0;
}