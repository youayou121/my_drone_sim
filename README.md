# 相关说明

仿真功能包第一次在编译的时候由于msgs的关系会失败，多编译几次即可。

## 1.仿真环境搭建

这里采用的是[ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner.git)里面自带的uav_simulator包，包括fast_planner也是用的这个仿真环境，所以我采用这个环境进行仿真。

## 2.ego_planner仿真

使用指令

```
roslaunch ego_planner ego_planner.launch
```

进行仿真实验，用rviz里的2DNavGoal发送目标点，不过这里的目标点都是二维目标点，即只有x，y，z则为0。

## 3.apf_local_planner仿真

apf_local_planner的源代码从[Prometheus](https://github.com/amov-lab/Prometheus.git)仿真平台获取，通过阅读其代码，该算法是直接通过目标信息和局部障碍物信息（二维laser或三维点云)来生成速度指令。由于仿真平台里给的局部点云仿真是模拟深度相机，而且点云坐标都是全局点云坐标。

我在这里添加了一个全局点云转局部点云的节点，该节点通过tf获取world和base的转换关系，再听过pcl_ros自带的方法转换。这样apf就可以获得局部点云信息。

可通过下方代码直接进行仿真。

```
roslaunch apf_local_planner simple_run.launch
```

由于直接通过目标点、局部点云信息和无人机位置信息获取的速度指令，这种方法有很多局限性，比如容易陷入局部最优，而且规划出来的速度也没有考虑到无人机的朝向，无人机都是平移，所以我下面将采用A*作为全局路径规划算法对路径进行提前规划。

## 4.a_star_global_planner算法与apf_local_planner融合

这里的a_star算法是从[github](https://github.com/Grizi-ju/Astar_JPS_Pathplanning_in_ROS)获取的

```
roslaunch global_planner demo_astar.launch
```

## 5.fast_planner仿真

将所有[fast_planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)的依赖包加上_fast后缀
