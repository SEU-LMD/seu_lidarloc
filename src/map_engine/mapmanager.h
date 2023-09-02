//
// Created by slam on 23-8-28.
//

#ifndef SERIALIZE_MAPMANAGER_H
#define SERIALIZE_MAPMANAGER_H

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
#include <vector>               //使用vector
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include "tic_toc.h"



typedef pcl::PointXYZ PointType;

const std::string map_read_path="/media/today/新加卷1/0831/mapout/";
const int up_grid_size=40;
const int up2down_num=2;  //up_grid_size/down_grid_size  要整数倍
const int laserCloudWidth = 4*up2down_num;  //x方向
const int laserCloudHeight = 5*up2down_num; //y方向
const int laserCloudNum = laserCloudWidth * laserCloudHeight;


typedef struct{
    std::string down_grid_index;
    int down_grid_num;
}Downgrid;


typedef struct {
    double x;
    double y;
    double z;
}Gnsspostion;










#endif //SERIALIZE_MAPMANAGER_H
