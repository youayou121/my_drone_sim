#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
ros::Publisher  pub_cloud;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr cloud2)
{
    static tf::TransformListener tf_listener;
    tf::StampedTransform trans_world_to_base;
    try
    {
        tf_listener.waitForTransform("base", "world", ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform("base","world", ros::Time(0), trans_world_to_base);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("look up trans_scan_to_base failed,due to:%s", ex.what());
        return;
    }
    sensor_msgs::PointCloud2 input = *cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud2, *pcl_local);
    pcl_ros::transformPointCloud(*pcl_local, *pcl_local, trans_world_to_base);
    pcl::CropBox<pcl::PointXYZ> crop;
    Eigen::Vector4f max_pos_map;
    Eigen::Vector4f min_pos_map;
    min_pos_map << -5, -5, -1, 1;
    max_pos_map << 5, 5, 1, 1;
    crop.setInputCloud(pcl_local);
    crop.setMin(min_pos_map);
    crop.setMax(max_pos_map);
    crop.setNegative(false);
    crop.filter(*pcl_local);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pcl_local, output);
    pub_cloud.publish(output);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_tf");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud = nh.subscribe("/pcl_render_node/cloud", 1,cloud_cb);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud",1);

    ros::spin();
    return 0;
}
