//
// Created by slam on 23-8-28.
//

#ifndef SERIALIZE_MAPMANAGER_H
#define SERIALIZE_MAPMANAGER_H
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <iostream>              //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/point_types.h>     //PCL中支持的点类型的头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>    //PCL点云类型库
#include <algorithm>            //找最大值和最小值的头文件
#include <cmath>                //向上取整数相关的
#include <fstream>              //文件输入输出的文件相关的
#include <pcl/common/transforms.h>  //点的变换相关的头文件
#include <pcl/kdtree/kdtree_flann.h> //kd tree
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>               //使用vector
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include "timer.h"
#include "config_helper.h"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP








typedef struct{
    std::string down_grid_index;
    int down_grid_num;
}Downgrid;


typedef struct {
    double x;
    double y;
    double z;
}Gnsspostion;



struct MyPointType{
    PCL_ADD_POINT4D;

    PCL_ADD_INTENSITY;

    int down_grid_index;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int, down_grid_index, down_grid_index)
)

typedef pcl::PointXYZI PointType;









#endif //SERIALIZE_MAPMANAGER_H
