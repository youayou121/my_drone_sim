# 相关说明

仿真功能包第一次在编译的时候由于msgs的关系会失败，多编译几次即可。

## 仿真环境搭建

这里采用的是[ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner.git)里面自带的uav_simulator包，包括fast_planner也是用的这个仿真环境，所以我采用这个环境进行仿真。

## ego_planner

ego_planner的输入信息包括里程计、点云数据以及给定的目标点坐标，使用起来还是比较方便的。

```
roslaunch ego_planner ego_planner.launch
```

我这里稍微修改了一下源码，之前发送的目标点坐标的z只能是0，修改过后可以用rviz里的3DNavGoal发送三维目标点坐标了进行仿真实验。

## fast_planner

由于ego_planner是[fast_planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)的一个改进版本，所以fast_planner和ego_planner有许多同名的功能包，如果需要在同一工作空间下编译，则需要将所有fast_planner的功能依赖包改名，且修改CMakeLists.txt的相关功能包依赖，以及package.xml的相关包说明也要改，这里采用的是将所有冲突的功能包加上_fast后缀。

### kino_replan

```
roslaunch plan_manage kino_replan.launch
```

### topo_replan

```
roslaunch plan_manage topo_replan.launch
```

同样修改了相关代码，可以接收3DNavGoal。

## simple_astar_planner

之所以叫[simple_astar_planner](https://github.com/Grizi-ju/Astar_JPS_Pathplanning_in_ROS.git)是因为这个算法相对于下面提到的astar算法可调整的参数较少，上面提到使ego planner可以接收到3d目标，由于ego planner没有对全局路径进行一个预规划（即全局路径是一条由起点到目标点的直线），我将把ego planner作为局部路径规划算法来使用。

```
roslaunch ego_planner ego_with_simple_astar.launch
```

我这里编写了一个pub_local_goal节点，将全局路径规划算法与ego_planner进行了一个结合，该节点将订阅全局路径规划算法发布的路径，并根据odom和膨胀点云信息发布一个局部目标点给ego_planner，该节点通用其他全局路径规划和局部路径规划算法，前提是全局路径规划算法有发布nav_msgs::Path类型的话题，如果没有发布这个话题，还得在源码里加上这么一个发布者，同时还得新加一个odom订阅节点用于实时更新路径规划的起点。

## jps_planner

这算法和[simple_astar_planner](https://github.com/Grizi-ju/Astar_JPS_Pathplanning_in_ROS.git)在一个功能包，但是该算法的规划时间很慢，且算法规划速度受全局点云数量和地图大小的影响。

```
roslaunch ego_planner ego_with_jps.launch
```

## astar_planner

该算法来自[Prometheus](https://github.com/amov-lab/Prometheus.git)开源仿真平台，相比上面的simple_astar，该算法可以修改的参数多一点，规划出来的全局路径也可以根据自己的要求进行修改，但所该算法的源代码中的全局代价地图会不断更新，这样将占用较大的计算资源，我这里修改了相应代码，只有初始化的时候更新全局地图，注意这里在打开拉launch之后需要等全局地图初始化完成方可进行路径规划操作。

```
roslaunch ego_planner ego_with_astar.launch
```

## hybrid_astar_planner

算法来自[Prometheus](https://github.com/amov-lab/Prometheus.git)开源仿真平台，相比上面的astar，该算法考虑到无人机的运动模型，规划出来的全局路径更丝滑，但所该算法与上面的astar有同样的问题，我这里也做了相关操作。

```
roslaunch ego_planner ego_with_hybrid_astar.launch
```

## rrt_star_planner

源码来自[github](https://github.com/KailinTong/Motion-Planning-for-Mobile-Robots/tree/master/hw_3/ros/catkin_ws/src/grid_path_searcher/src)

```
roslaunch ego_planner ego_with_rrt_star.launch
```

## apf_local_planner

源码来自[Prometheus](https://github.com/amov-lab/Prometheus.git)仿真平台，通过阅读其代码，该算法是直接通过目标信息和局部障碍物信息（二维laser或三维点云)来生成速度指令。由于仿真平台里给的局部点云仿真是模拟深度相机，而且点云坐标都是全局点云坐标。

```
roslaunch apf_local_planner simple_run.launch
```

我在这里添加了一个全局点云转局部点云的节点来模拟3D雷达得到的效果，该节点通过tf获取world和base的转换关系，再听过pcl_ros自带的方法转换。这样apf就可以获得局部点云信息。由于直接通过目标点、局部点云信息和无人机位置信息获取的速度指令，这种方法有很多局限性，比如容易陷入局部最优，而且规划出来的速度也没有考虑到无人机的朝向，无人机都是平移，所以我下面将采用混合A*作为全局路径规划算法对路径进行提前规划。

```
roslaunch apf_local_planner apf_with_hybrid_astar.launch
```

要注意打开launch之后会有一段时间用于初始化全局地图，如果太卡在launch文件中适当减小膨胀半径，由于混合A*是三维的搜索方法，为了防止出现垂直方向的路径规划，我在这里将全局地图的z方向拓展很多，而且没有设置circle，以防止出现垂直方向的路径规划。
