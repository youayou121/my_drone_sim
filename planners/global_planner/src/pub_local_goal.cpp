#include"ros/ros.h"
#include"nav_msgs/Path.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/PoseStamped.h"
#include"sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<vector>
using namespace std;
vector<geometry_msgs::PoseStamped> global_path;
geometry_msgs::Pose current_pose;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher goal_pub;
int pre_goal_index = -1;
bool reach_goal = false;
int look_ahead_distance = 7.0;
bool is_point_cloud_in_range(const pcl::PointXYZ point, float range)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pcl_global);

    std::vector<int> pointIndices;
    std::vector<float> pointDistances;

    // 在给定范围内搜索最近邻点
    if (kdtree.radiusSearch(point, range, pointIndices, pointDistances) > 0)
    {
        return true; // 范围内存在点云
    }

    return false; // 范围内不存在点云
}

double distance(geometry_msgs::Pose pos1,geometry_msgs::Pose pos2)
{
    return (pos1.position.x-pos2.position.x)*(pos1.position.x-pos2.position.x)+(pos1.position.y-pos2.position.y)*(pos1.position.y-pos2.position.y)+(pos1.position.z-pos2.position.z)*(pos1.position.z-pos2.position.z);
}

int find_next_index(geometry_msgs::Pose pose)
{
    double min_dist = 100;
    int min_dist_index = global_path.size()-1;
    for(int i = 0;i<global_path.size();i++)
    {
        geometry_msgs::Pose path_pos = global_path[i].pose;        
        if(distance(path_pos,pose)<min_dist)
        {
            min_dist_index = i;
            min_dist = distance(path_pos,pose);
        }
    }
    int current_goal_index = min_dist_index;
    while(current_goal_index<global_path.size())
    {
        pcl::PointXYZ point;
        point.x = global_path[current_goal_index].pose.position.x;
        point.y = global_path[current_goal_index].pose.position.y;
        point.z = global_path[current_goal_index].pose.position.z;
        if (is_point_cloud_in_range(point,0.1))
        {
            if(current_goal_index<global_path.size()-1)
            {
                current_goal_index++;
            }
        }
        if(distance(global_path[current_goal_index].pose,current_pose)>look_ahead_distance||current_goal_index==global_path.size()-1)
        {
            break;
        }
        current_goal_index++;
    }
    return current_goal_index;
}

void path_cb(const nav_msgs::PathConstPtr path)
{
    reach_goal = false;
    cout<<"path_cb"<<endl;
    global_path.resize(0);
    global_path = path->poses;
}

void odom_cb(const nav_msgs::OdometryConstPtr odom)
{
    current_pose = odom->pose.pose;
    if(!global_path.empty()&&!reach_goal)
    {
        if(distance(current_pose,global_path.back().pose)<0.5)
        {
            reach_goal = true;
        }
        else 
        {
            cout << global_path.size() << endl;
            geometry_msgs::PoseStamped current_goal;
            int goal_index = find_next_index(current_pose);
            if (pre_goal_index != goal_index)
            {
                current_goal = global_path[goal_index];
                goal_pub.publish(current_goal);
                pre_goal_index = goal_index;
            }
        }
        

    }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr cloud2)
{
    pcl::fromROSMsg(*cloud2, *pcl_global);
    // cout<<pcl_global->size()<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pub_local_goal_node");
    ros::NodeHandle nh;
    ros::Subscriber path_sub = nh.subscribe("/global_path",1,path_cb);
    ros::Subscriber odom_sub = nh.subscribe("/odom",1,odom_cb);
    ros::Subscriber cloud_sub = nh.subscribe("/global_cloud",1,cloud_cb);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_goal",1);
    ros::spin();
    return 0;
}
