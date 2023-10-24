#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
using namespace std;
class point_cloud2_frame_transformer{
    public:
        point_cloud2_frame_transformer(char* target_frame,char* target_topic_name,char* pc_topic_name,char* LR_obst_PC_name,Eigen::Vector4f &max_pos,Eigen::Vector4f &min_pos,Eigen::Vector4f &angular_max_pos,Eigen::Vector4f &angular_min_pos,float laser_beam_radius,float tray_distance,float ignore_radius);

    private:
        void point_cloud_callback_(const sensor_msgs::PointCloud2ConstPtr &msg);
        bool laser_beam_check_service_callback_(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);
        void path_cb(const geometry_msgs::PoseArray pose_arr);  // 全局路径回调函数
        float distance(float x1, float y1, float x2, float y2); // 计算两点之间直线距离
        int in_local_path(float x, float y,bool &if_head);  // 判断障碍物是否在两条扫过的路径上，并返回对应的索引
        int find_closest_index(float x, float y); // 通过global_pose_arr和自己的位置判断agv是否在路径上，如果在则返回索引，如果不在则返回-1
        void pub_obstacle_region(int global_path_index); //发布停障区域方法
        ros::NodeHandle nh_;
        
        ros::Subscriber pc2_suber_;
        ros::Publisher transformer_pc2_puber_;
        ros::Publisher transformer_anglar_pc_puber_;
        ros::Publisher closest_distance_puber_;
        ros::Subscriber sub_global_path;     // 订阅全局路径保存到global_pose_arr;
        ros::Publisher pub_local_path_head_; // 发布剩余的局部路径
        ros::Publisher pub_local_path_rear_; // 发布剩余的局部路径
        ros::Publisher pub_obstacle_region_; // 发布停障区域publisher
        std::string target_frame_name_;
        tf::TransformListener tf_listener;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angular_final {new pcl::PointCloud<pcl::PointXYZ>};
        tf::StampedTransform transform;
        sensor_msgs::PointCloud2 PC_transformed;
        pcl::CropBox<pcl::PointXYZ> cropbox,angular_cropbox;
        ros::ServiceServer beam_distance_detection_server_;
        Eigen::Vector4f crop_box_max,crop_box_min,angular_crop_box_max,angular_crop_box_min;

        float beam_distance_;
        float laser_beam_radius_;
        float tray_distance_;
        float blind_zone_radius_;
        float rotate_radius_;//旋转半径
        sensor_msgs::PointCloud2 PC_out,angular_PC_out;
        int seq = 0;
        nav_msgs::Path local_path_head;                // agv头部的剩余局部路径
        nav_msgs::Path local_path_rear;                // agv尾部的剩余局部路径
        geometry_msgs::PoseArray global_pose_arr;      // 存储订阅到的全局路径
        geometry_msgs::PolygonStamped obstacle_region; // 用于发布停障区域
        int global_path_index = -1;                    // agv在全局路径下的索引，即离全局路径里面最近的点，如果最小距离大于in_path_threshold则返回-1
        bool flag_beizer = false;                      // 用于记录当前运动是否是贝塞尔运动
        bool if_omni = false;
        float head_radius = 0.4;        // agv头部碰撞半径
        float rear_radius = 0.3;        // agv尾部碰撞半径
        float in_path_threshold = 0.15; // 用于判断agv是否在全局路径里的阈值
        float ahead_length = 0.7;       // agv尾部与头部碰撞半径的圆心距离
        float poly_end_slow = 0;        // 保存obstacle_slow_distance
        float poly_end_stop = 0;        // 保存obstacle_stop_distance
        float robot_front_length = 1.0;
        float obstacle_region_width = 0.525; // 直行停障可视化区域宽度的1/2，即clip_end_y
        float stop_distance_near_goal_obstacle = 0.2;
        string current_orient_frame = "base_link"; // 贝塞尔停障区域基于的frame_id
        vector<float> path_distance;               // 保存agv剩余路径长度
};

bool point_cloud2_frame_transformer::laser_beam_check_service_callback_(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
    ROS_INFO("received distance measuring request.");
    
    if(beam_distance_>tray_distance_ or beam_distance_ == -1){
        res.success = false;
        res.message.append("no tray ahead, dock in allowed");
    }else{
        res.success = true;
        res.message.append("tray is in front of the vehicle, do not dock in.");
    }
    return true;
}

point_cloud2_frame_transformer::point_cloud2_frame_transformer(char* target_frame,
                                                                char* target_topic_name,
                                                                char* pc_topic_name,
                                                                char* LR_obst_PC_name,
                                                                Eigen::Vector4f &max_pos,
                                                                Eigen::Vector4f &min_pos,
                                                                Eigen::Vector4f &angular_max_pos,
                                                                Eigen::Vector4f &angular_min_pos,
                                                                float laser_beam_radius,
                                                                float tray_distance,
                                                                float ignore_radius){
    ros::NodeHandle private_nh_("~");

    target_frame_name_.append(target_frame);
    pub_local_path_head_ = nh_.advertise<nav_msgs::Path>("/local_path_head", 1);
    pub_local_path_rear_ = nh_.advertise<nav_msgs::Path>("/local_path_rear", 1);
    pub_obstacle_region_ = nh_.advertise<geometry_msgs::PolygonStamped>("/obstacle_region",1);
    sub_global_path = nh_.subscribe<geometry_msgs::PoseArray>("/dock_pose_arr", 1, &point_cloud2_frame_transformer::path_cb, this);
    pc2_suber_ = nh_.subscribe<sensor_msgs::PointCloud2>(pc_topic_name,10,&point_cloud2_frame_transformer::point_cloud_callback_,this);
    transformer_pc2_puber_ = nh_.advertise<sensor_msgs::PointCloud2>(target_topic_name,10);
    transformer_anglar_pc_puber_ = nh_.advertise<sensor_msgs::PointCloud2>(LR_obst_PC_name,10);
    
    closest_distance_puber_ = nh_.advertise<std_msgs::Float32MultiArray>("/obstacle_distance",1);
    beam_distance_detection_server_ = nh_.advertiseService("/laser_beam_obstacles_checking",&point_cloud2_frame_transformer::laser_beam_check_service_callback_,this);

    beam_distance_ = -1; 
    laser_beam_radius_ = laser_beam_radius;
    tray_distance_ = tray_distance;

    ROS_INFO("%f %f %f",beam_distance_,laser_beam_radius_,tray_distance_);
    cropbox.setMax(max_pos);
    cropbox.setMin(min_pos);
    angular_cropbox.setMax(angular_max_pos);
    angular_cropbox.setMin(angular_min_pos);

    blind_zone_radius_ = ignore_radius;
    ros::param::get("rotate_radius",rotate_radius_);

    obstacle_region_width = max_pos.y();
    ros::param::get("obstacle_stop_distance",poly_end_stop);
    ros::param::get("obstacle_slow_distance",poly_end_slow);
    ros::param::get("rotate_radius", rotate_radius_);
    ros::param::get("head_radius",head_radius);
    ros::param::get("rear_radius",rear_radius);
    ros::param::get("in_path_threshold",in_path_threshold);
    ros::param::get("ahead_length",ahead_length);
    ros::param::get("robot_front_length",robot_front_length);
    ros::param::get("stop_distance_near_goal_obstacle",stop_distance_near_goal_obstacle);
    ros::param::get("if_omni",if_omni);
    if(if_omni)
    {
        current_orient_frame = "current_orient";
    }
    ROS_INFO("initiallization done, start detecting obstacle.");
}

void point_cloud2_frame_transformer::point_cloud_callback_(const sensor_msgs::PointCloud2ConstPtr &msg){
    
    std_msgs::Float32MultiArray distance_to_pub;
    //通过tf获取agv的在地图坐标下的位姿
    tf::StampedTransform trans_base_to_map;
    tf::StampedTransform trans_scan_to_base;
    try
    {
        tf_listener.waitForTransform("map", current_orient_frame, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform("map", current_orient_frame, ros::Time(0), trans_base_to_map);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("look up trans_scan_to_base failed,due to:%s", ex.what());
        return;
    }
    try
    {
        tf_listener.waitForTransform(current_orient_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(current_orient_frame, msg->header.frame_id, ros::Time(0), trans_scan_to_base);
    }
    catch (tf::TransformException &ex) {
		ROS_ERROR("look up trans_scan_to_base failed,due to:%s",ex.what());
		return ;
	}
    float map_x = trans_base_to_map.getOrigin().getX();
    float map_y = trans_base_to_map.getOrigin().getY();
    global_path_index = find_closest_index(map_x, map_y);  //找到离agv base_link最近的全局路径点
    sensor_msgs::PointCloud2 PC_initial = *msg;
    pcl_ros::transformPointCloud(current_orient_frame,trans_scan_to_base,PC_initial,PC_transformed);
    pcl::fromROSMsg(PC_transformed,*cloud_in);
    //判断agv在不在路径上
    if(global_path_index>=0)
    {
        //如果在路径上
        local_path_head.header.frame_id = current_orient_frame;
        local_path_head.poses.resize(0);
        local_path_head.header.seq = seq;
        local_path_rear.header.frame_id = current_orient_frame;
        local_path_rear.poses.resize(0);
        local_path_rear.header.seq = seq;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = current_orient_frame;
        
        if (global_pose_arr.poses.size() > 0)
        {
            //通过订阅到的全局路径分别计算agv头部和尾部扫过的路径，
            //这两者的都是通过全局路径平移而来，并将全局坐标转换为基于baselink的局部坐标
            for (int j = global_path_index; j < global_pose_arr.poses.size(); j++)
            {
                tf::Vector3 map_pose(
                    global_pose_arr.poses[j].position.x,
                    global_pose_arr.poses[j].position.y,
                    global_pose_arr.poses[j].position.z);
                tf::Vector3 local_pose = trans_base_to_map.inverse() * map_pose;
                pose.pose.position.x = local_pose.getX();
                pose.pose.position.y = local_pose.getY();
                pose.header.stamp = ros::Time::now();
                local_path_rear.poses.push_back(pose);
                pose.pose.position.x = local_pose.getX()+ahead_length;
                local_path_head.poses.push_back(pose);
            }
            //剩余路径长度初始化
            path_distance.resize(local_path_head.poses.size());
            float path_sum = 0;
            for (int j = 0; j < local_path_head.poses.size(); j++)
            {
                if (j != 0)
                {
                    float path_distance_j = distance(
                        local_path_rear.poses[j - 1].pose.position.x,
                        local_path_rear.poses[j - 1].pose.position.y,
                        local_path_rear.poses[j].pose.position.x,
                        local_path_rear.poses[j].pose.position.y);
                    path_sum += path_distance_j;
                }
                path_distance[j] = path_sum;
            }
        }
        local_path_head.header.stamp = ros::Time::now();
        local_path_rear.header.stamp = ros::Time::now();
        pub_local_path_head_.publish(local_path_head);
        pub_local_path_rear_.publish(local_path_rear);
        pub_obstacle_region(global_path_index);
    }else
    {
        pub_obstacle_region(global_path_index);
    }
    // 如果agv在路径上且agv做曲线运动，则只考虑路径上的障碍物
    if (global_path_index >= 0&&flag_beizer) 
    {
        float min_obstacle_distance = -1;
        seq++;
        
        if (cloud_in->points.size() > 0)
        {
            min_obstacle_distance = -1;
            //遍历激光点云并且判断该点在不在路径上，并判断在头部扫过的路径上还是尾部扫过的路径上，返回相应的path_index
            for (int i = 0; i < cloud_in->points.size(); i++)
            {
                float pcl_x = cloud_in->points[i].x;
                float pcl_y = cloud_in->points[i].y;
                if (local_path_head.poses.size() > 0)
                {
                    bool if_head = false;
                    int path_index = in_local_path(pcl_x, pcl_y,if_head);
                    if (path_index > 0)
                    {
                        if(path_distance[path_index]<min_obstacle_distance||min_obstacle_distance<0)
                        {
                            min_obstacle_distance = path_distance[path_index];
                        }
                    }
                }
            }
        }
        else
        {
            min_obstacle_distance = -1;
        }
        distance_to_pub.data.push_back(min_obstacle_distance);
    }
     // 如果不在路径上或者agv做直线运动，使用正前方障碍物区域
    else
    {
        // point cloud process
        // front obstacle detector

        // TODO: use the same cloud for obstacle checking
        cropbox.setInputCloud(cloud_in);
        cropbox.filter(*cloud_final);

        
        if (cloud_final->size() != 0)
        {
            float beam_distance = 1000;
            float closest_distance;
            float distance;
            for (int i = 0; i < cloud_final->size(); i++)
            {
                if (i == 0)
                {
                    closest_distance = cloud_final->points[i].x; // sqrt(pow(cloud_final->points[i].x,2)+pow(cloud_final->points[i].y,2));
                    if (abs(cloud_final->points[i].y) < laser_beam_radius_)
                    {
                        beam_distance = closest_distance;
                    }
                }
                else
                {
                    distance = cloud_final->points[i].x; // sqrt(pow(cloud_final->points[i].x,2)+pow(cloud_final->points[i].y,2));
                    if (distance < closest_distance)
                        closest_distance = distance;
                    if (abs(cloud_final->points[i].y) < laser_beam_radius_)
                    {
                        if (distance < beam_distance)
                            beam_distance = distance;
                    }
                }
            }
            beam_distance_ = beam_distance;
            distance_to_pub.data.push_back(closest_distance-robot_front_length);
            // ROS_INFO("obstacle distance:%f",beam_distance_);
        }
        else
        {
            beam_distance_ = -1;
            distance_to_pub.data.push_back(-1);
        }
    }
    // rotation obstacle detection
    angular_cropbox.setInputCloud(cloud_in);
    angular_cropbox.filter(*cloud_angular_final);
    //点云过滤
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    if(cloud_angular_final->size()!=0){
        
        float distanceL = -1,distanceR = -1;
        float closest_distanceL = distanceL,closest_distanceR = distanceR;
        for(int i=0;i<cloud_angular_final->size();i++){
            // ROS_INFO("%f %f %f",cloud_angular_final->points[i].x,cloud_angular_final->points[i].y,cloud_angular_final->points[i].z);
            // right detector
            float distance = sqrt(pow(cloud_angular_final->points[i].x, 2) + pow(cloud_angular_final->points[i].y, 2));
            if (distance < rotate_radius_)
            {
                inliers->indices.push_back(i);//只有小于旋转半径的点才加入到左右避障点云中
                if(cloud_angular_final->points[i].y<0 && distanceR == -1){
                    distanceR = abs(cloud_angular_final->points[i].y);
                    closest_distanceR = distanceR;
                }else if(cloud_angular_final->points[i].y<0 && distanceR != -1){
                    distanceR = abs(cloud_angular_final->points[i].y);
                    if(distanceR < closest_distanceR && distanceR > blind_zone_radius_)
                        closest_distanceR = distanceR;
                }   

                if(cloud_angular_final->points[i].y>0 && distanceL == -1){
                    distanceL = abs(cloud_angular_final->points[i].y);
                    closest_distanceL = distanceL;
                }else if(cloud_angular_final->points[i].y>=0 && distanceL == -1){
                    distanceL = abs(cloud_angular_final->points[i].y);
                    if(distanceL < closest_distanceL && distanceL > blind_zone_radius_)
                        closest_distanceL = distanceL;
                }
            }
        }
        distance_to_pub.data.push_back(closest_distanceL);
        distance_to_pub.data.push_back(closest_distanceR);
        //ROS_INFO("obstacle distance:%f",beam_distance_);
    }else{
        beam_distance_ = -1;
        distance_to_pub.data.push_back(-1);
        distance_to_pub.data.push_back(-1);
    }
    // point cloud process done
    //点云过滤
    extract.setInputCloud(cloud_angular_final);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_angular_final);

    pcl::toROSMsg(*cloud_final,PC_out);
    pcl::toROSMsg(*cloud_angular_final,angular_PC_out);
    
    transformer_pc2_puber_.publish(PC_out);
    transformer_anglar_pc_puber_.publish(angular_PC_out);

    closest_distance_puber_.publish(distance_to_pub);
}

float point_cloud2_frame_transformer::distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int point_cloud2_frame_transformer::in_local_path(float x, float y,bool &if_head)
{
    int index = 0;
    for (int i = 0; i < local_path_head.poses.size(); i++)
    {
        float dist_front = distance(x, y, local_path_head.poses[i].pose.position.x, local_path_head.poses[i].pose.position.y);
        
        if(dist_front < head_radius)
        {
            index = i;
            if_head = true;
            return index;
        }
    }
    for (int i = 0; i < local_path_rear.poses.size(); i++)
    {
        float dist_rear = distance(x, y, local_path_rear.poses[i].pose.position.x, local_path_rear.poses[i].pose.position.y);
        if (dist_rear < rear_radius)
        {
            index = i;
            if_head = false;
            return index;
        }
    }

    return -1;

}

void point_cloud2_frame_transformer::path_cb(const geometry_msgs::PoseArray pose_arr)
{
    global_pose_arr = pose_arr;
}

int point_cloud2_frame_transformer::find_closest_index(float x, float y)
{
    float min_dist = 100;
    int index = 0;
    if (global_pose_arr.poses.size() > 0)
    {
        for (int i = 0; i < global_pose_arr.poses.size(); i++)
        {
            float dist = distance(x, y, global_pose_arr.poses[i].position.x, global_pose_arr.poses[i].position.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                index = i;
            }
        }
    }
    if (min_dist < in_path_threshold)
    {
        return index;
    }
    else
    {
        return -1;
    }
}

void point_cloud2_frame_transformer::pub_obstacle_region(int global_path_index)
{
    obstacle_region.header.frame_id = current_orient_frame;
    obstacle_region.header.seq = seq;
    obstacle_region.header.stamp = ros::Time::now();
    obstacle_region.polygon.points.resize(0);
    if(global_path_index<0)
    {
        geometry_msgs::Point32 point;
        //右下
        point.x = robot_front_length;
        point.y = obstacle_region_width;
        obstacle_region.polygon.points.push_back(point);
        //右上slow
        point.x = robot_front_length + poly_end_slow;
        obstacle_region.polygon.points.push_back(point);
        //左上slow
        point.y = -obstacle_region_width;
        obstacle_region.polygon.points.push_back(point);
        //左上stop
        point.x = robot_front_length + poly_end_stop;
        obstacle_region.polygon.points.push_back(point);
        //右上stop
        point.y = obstacle_region_width;
        obstacle_region.polygon.points.push_back(point);
        //左上stop
        point.y = -obstacle_region_width;
        obstacle_region.polygon.points.push_back(point);
        //左下
        point.x = robot_front_length;
        obstacle_region.polygon.points.push_back(point);
        pub_obstacle_region_.publish(obstacle_region);
    }
    else
    {
        int poly_length_slow_rear = 0;
        int poly_length_slow_head = 0;
        int poly_length_stop_rear = 0;
        int poly_length_stop_head = 0;
        // 剪切固定路径长度
        for (int i = 0; i < path_distance.size(); i++)
        {
            if (path_distance[i] < (poly_end_stop))
            {
                poly_length_stop_rear = i;
            }
            if (path_distance[i] < (poly_end_slow))
            {
                poly_length_slow_rear = i;
            }
            if (path_distance[i] < poly_end_stop + ahead_length)
            {
                poly_length_stop_head = i;
            }
            if (path_distance[i] < poly_end_slow + ahead_length)
            {
                poly_length_slow_head = i;
            }
        }
        // 判断是否是贝塞尔运动
        if (abs(local_path_rear.poses[0].pose.position.y - local_path_rear.poses[poly_length_slow_rear - 1].pose.position.y) > rear_radius / 2 && poly_length_slow_rear)
        {
            flag_beizer = true;
        }
        else
        {
            flag_beizer = false;
        }
        geometry_msgs::Point32 point;
        int clockwise = 1;
        if (flag_beizer)
        {
            // 如果是贝塞尔运动
            // 判断是否是顺时针旋转，用于优化可视化效果，由于车宽的原因曲线运动外端需要更长的路径
            clockwise = local_path_rear.poses[0].pose.position.y >
                                local_path_rear.poses[poly_length_slow_rear - 1].pose.position.y
                            ? 1
                            : -1;
            //外head stop下上
            for (int i = 0; i < poly_length_stop_rear; i++)
            {
                point.x = local_path_head.poses[i].pose.position.x + ahead_length;
                point.y = local_path_head.poses[i].pose.position.y + clockwise * rear_radius;
                obstacle_region.polygon.points.push_back(point);
            }
            //内rear stop上
            point.x = local_path_rear.poses[poly_length_stop_head-1].pose.position.x;
            point.y = local_path_rear.poses[poly_length_stop_head-1].pose.position.y - clockwise * rear_radius;
            obstacle_region.polygon.points.push_back(point);
            //外head stop上
            point.x = local_path_head.poses[poly_length_stop_rear-1].pose.position.x + ahead_length;
            point.y = local_path_head.poses[poly_length_stop_rear-1].pose.position.y + clockwise * rear_radius;
            obstacle_region.polygon.points.push_back(point);
            //外head stop-slow
            for (int i = poly_length_stop_rear-1; i < poly_length_slow_rear; i++)
            {
                point.x = local_path_head.poses[i].pose.position.x + ahead_length;
                point.y = local_path_head.poses[i].pose.position.y + clockwise * rear_radius;
                obstacle_region.polygon.points.push_back(point);
            }
            //内rear slow上下
            for (int i = poly_length_slow_head-1; i >=0; i--)
            {
                point.x = local_path_rear.poses[i].pose.position.x;
                point.y = local_path_rear.poses[i].pose.position.y - clockwise * rear_radius;
                obstacle_region.polygon.points.push_back(point);
            }
            //外head stop下
            point.x = local_path_head.poses[0].pose.position.x;
            point.y = local_path_head.poses[0].pose.position.y + clockwise * rear_radius;
            obstacle_region.polygon.points.push_back(point);

        }
        else
        {
            // 直行的停障和减速框
            if (poly_length_slow_rear)
            {
                float forward = 0.1;
                if (local_path_rear.poses[poly_length_slow_rear - 1].pose.position.x -
                        local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x <
                    0.1)
                {
                    forward = stop_distance_near_goal_obstacle;
                }
                // 停障框
                //左下
                point.x = robot_front_length;
                point.y = -obstacle_region_width;
                obstacle_region.polygon.points.push_back(point);
                //左上stop
                point.x = robot_front_length + forward + local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x;
                // point.y = local_path_rear.poses[poly_length_stop_rear - 1].pose.position.y - obstacle_region_width;
                obstacle_region.polygon.points.push_back(point);
                //右上stop
                // point.x = robot_front_length + forward + local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x;
                point.y = obstacle_region_width;
                obstacle_region.polygon.points.push_back(point);
                //右下
                point.x = robot_front_length;
                // point.y = local_path_rear.poses[0].pose.position.y + obstacle_region_width;
                obstacle_region.polygon.points.push_back(point);
                //左下
                // point.x = robot_front_length;
                point.y = -obstacle_region_width;
                obstacle_region.polygon.points.push_back(point);

                if (local_path_rear.poses[poly_length_slow_rear - 1].pose.position.x -
                        local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x >
                    0.1)
                {
                    // 减速框
                    //左上stop
                    point.x = robot_front_length + forward + local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x;
                    // point.y = local_path_rear.poses[poly_length_stop_rear - 1].pose.position.y - obstacle_region_width;
                    obstacle_region.polygon.points.push_back(point);
                    //左上slow
                    point.x = robot_front_length + local_path_rear.poses[poly_length_slow_rear - 1].pose.position.x;
                    // point.y = local_path_rear.poses[poly_length_slow_rear - 1].pose.position.y - obstacle_region_width;
                    obstacle_region.polygon.points.push_back(point);
                    //右上slow
                    // point.x = robot_front_length + local_path_rear.poses[poly_length_slow_rear - 1].pose.position.x;
                    point.y = obstacle_region_width;
                    obstacle_region.polygon.points.push_back(point);
                    //右上stop
                    point.x = robot_front_length + forward + local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x;
                    point.y = obstacle_region_width;
                    obstacle_region.polygon.points.push_back(point);
                    //左下stop
                    // point.x = robot_front_length + forward + local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x;
                    point.y = -obstacle_region_width;
                    obstacle_region.polygon.points.push_back(point);
                }
            }
        }
        // 如果是倒车运动就resize polygon并且不发布polygon
        if (local_path_rear.poses[0].pose.position.x > local_path_rear.poses[poly_length_stop_rear - 1].pose.position.x)
        {
            obstacle_region.polygon.points.resize(0);
        }
        else
        {
            pub_obstacle_region_.publish(obstacle_region);
        }
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"obstacle_detector_node");
    Eigen::Vector4f max_pos;
    Eigen::Vector4f min_pos;
    Eigen::Vector4f angular_max_pos;
    Eigen::Vector4f angular_min_pos;
    float laser_beam_radius = 0;
    float tray_distance = 0;
    float ignore_radius = 0;

    ros::NodeHandle private_nh;
    float clip_x_end,clip_y_end,clip_z_end,clip_x_start;
    float radius_clip_x_start,radius_clip_y_end,radius_clip_z_end,radius_clip_x_end;
    private_nh.param("clip_x_start",clip_x_start,0.3f);
    private_nh.param("clip_x_end",clip_x_end,2.3f);
    private_nh.param("clip_y_end",clip_y_end,0.3f);
    private_nh.param("clip_z_end",clip_z_end,0.0f);
    private_nh.param("narrow_y_end",laser_beam_radius,0.05f);
    private_nh.param("tray_distance",tray_distance,2.5f);
    private_nh.param("radius_clip_x_start",radius_clip_x_start,0.5f);
    private_nh.param("radius_clip_x_end",radius_clip_x_end,2.0f);
    private_nh.param("radius_clip_y_end",radius_clip_y_end,1.0f);
    private_nh.param("radius_clip_z_end",radius_clip_z_end,0.0f);
    private_nh.param("ignore_radius",ignore_radius,0.1f);

    max_pos << clip_x_end,clip_y_end,clip_z_end,1;
    min_pos << clip_x_start,-clip_y_end,clip_z_end,1;
    angular_max_pos << radius_clip_x_end,radius_clip_y_end,radius_clip_z_end,1;
    angular_min_pos << radius_clip_x_start,-radius_clip_y_end,radius_clip_z_end,1;

    std::cout<< angular_max_pos << std::endl << angular_min_pos << std::endl;


    point_cloud2_frame_transformer pc_transformer("base_link","/pc_in_base","/cloud","/obstcale_LR_PC",max_pos,min_pos,angular_max_pos,angular_min_pos,laser_beam_radius,tray_distance,ignore_radius);
    ros::spin();
    return 0;
}



