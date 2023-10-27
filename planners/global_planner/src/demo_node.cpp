#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

#define _use_jps  0    //0 -> 只使用Ａ*, 1 -> 使用JPS

namespace backward {
backward::SignalHandling sh;
}


double _resolution, _inv_resolution, _cloud_margin;

double _x_size, _y_size, _z_size;    

bool _has_map   = false;

Vector3d _start_pt;

Vector3d _map_lower, _map_upper;

int _max_x_id, _max_y_id, _max_z_id;


ros::Subscriber _map_sub, _pts_sub;
ros::Subscriber odom_sub;
ros::Publisher pub_local_goal;
ros::Publisher pub_path;
vector<Vector3d> global_path;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder *_jps_path_finder = new JPSPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );

void visVisitedNode( vector<Vector3d> nodes );

void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt);  
    nav_msgs::Path path;  
    for(Vector3d pos:global_path)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "world";
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = pos(0);
        ps.pose.position.y = pos(1);
        ps.pose.position.z = pos(2);
        ps.pose.orientation.x = 0;
        ps.pose.orientation.y = 0;
        ps.pose.orientation.z = 0;
        ps.pose.orientation.w = 1;
        path.poses.push_back(ps);
    }
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    pub_path.publish(path);
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);
     
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{


    if(_use_jps)
    {
        // 调用JPS算法
        _jps_path_finder->JPSGraphSearch(start_pt, target_pt);

        //获得最优路径和访问的节点
        auto visited_nodes = _jps_path_finder->getVisitedNodes();
        global_path = _jps_path_finder->getPath();
        //可视化
        visGridPath(global_path, _use_jps);
        visVisitedNode(visited_nodes);

        //重置栅格地图,为下次调用做准备
        _jps_path_finder->resetUsedGrids();
    }
    else
    {
        _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

        global_path = _astar_path_finder->getPath();
        auto visited_nodes = _astar_path_finder->getVisitedNodes();

        visGridPath(global_path, false);

        visVisitedNode(visited_nodes);

        _astar_path_finder->resetUsedGrids();
    }



}

double distance(geometry_msgs::Point pos1,geometry_msgs::Point pos2)
{
    return (pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y)+(pos1.z-pos2.z)*(pos1.z-pos2.z);
}

int find_next_index(geometry_msgs::Point pose)
{
    double min_dist = 100;
    int min_dist_index = 0;
    for(int i = 0;i<global_path.size();i++)
    {
        geometry_msgs::Point path_pos;
        path_pos.x = global_path[i](0);
        path_pos.y = global_path[i](1);
        path_pos.z = global_path[i](2);
        if(distance(path_pos,pose)<min_dist)
        {
            min_dist_index = i;
            min_dist = distance(path_pos,pose);
        }
    }
    if(min_dist_index+6<global_path.size()-1)
    {
        return min_dist_index+6;
    }
    else
    {
        return global_path.size()-1;
    }
}

void odom_cb(const nav_msgs::OdometryConstPtr& odom)
{
    _start_pt(0) = odom->pose.pose.position.x;
    _start_pt(1) = odom->pose.pose.position.y;
    _start_pt(2) = odom->pose.pose.position.z;
    // std::cout<<"odom_cb"<<endl;
    if(global_path.size()>0)
    {
        geometry_msgs::PoseStamped current_goal;
        current_goal.header.frame_id = "world";
        current_goal.header.stamp = ros::Time::now();
        current_goal.pose.orientation = odom->pose.pose.orientation;
        int next_index = find_next_index(odom->pose.pose.position);
        if(next_index==global_path.size()-1)
        {
            return;
        }
        current_goal.pose.position.x = global_path[next_index](0);
        current_goal.pose.position.y = global_path[next_index](1);
        current_goal.pose.position.z = global_path[next_index](2);
        // pub_local_goal.publish(current_goal);
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    odom_sub = nh.subscribe("odom",1,odom_cb);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("current_goal",1);
    pub_path = nh.advertise<nav_msgs::Path>("/global_path",1);
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);


    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _astar_path_finder  = new AstarPathFinder(); 

    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    _jps_path_finder = new JPSPathFinder();
    _jps_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {		
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }
 
    delete _astar_path_finder;
    delete _jps_path_finder;
    return 0;
}

void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;     //jps  black 
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;      //astar  white
        node_vis.color.r = 1.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 1.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}
