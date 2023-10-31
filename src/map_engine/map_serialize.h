//
// Created by today on 23-8-18.
//

#ifndef SRC_LOADMAP_H
#define SRC_LOADMAP_H

#define PCL_NO_PRECOMPILE


#include <iostream>              //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/point_types.h>     //PCL中支持的点类型的头文件
#include <pcl/point_cloud.h>
#include <algorithm>            //找最大值和最小值的头文件
#include <cmath>                //向上取整数相关的
#include <fstream>              //文件输入输出的文件相关的
#include <pcl/common/transforms.h>  //点的变换相关的头文件
#include "Eigen/Core"           //eigen
#include "Eigen/Geometry"       //Isometry3d
#include "config_helper.h"
#include <pcl/filters/uniform_sampling.h>
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP


struct Tum{
    double timestamp; // 时间戳
    double tx, ty, tz; // 平移向量
    double qx, qy, qz, qw; // 四元数表示的旋转
};








#endif //SRC_LOADMAP_H
