#include<ros/ros.h>
#include <tf/transform_listener.h>
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
     sensor_msgs::PointCloud2 output;
    pcl_ros::transformPointCloud("base",trans_world_to_base,input,output);
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
