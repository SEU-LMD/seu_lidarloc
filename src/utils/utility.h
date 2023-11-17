#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE
#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))
//TODO 1111 remove from ros!!!!


#include "opencv2/opencv.hpp"   // for opencv4
#include "Eigen/Dense"


#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/impl/search.hpp"
#include "pcl/range_image/range_image.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/registration/icp.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/crop_box.h"
#include "pcl/conversions.h"


#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <ctime>
#include <cstdlib>
#include <chrono>

#include "config_helper.h"
using namespace std;

typedef pcl::PointXYZI PointType;

struct PointXYZIRPYT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIRPYT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                         time))
typedef PointXYZIRPYT PointTypePose;


float pointDistance(PointType p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

Eigen::Matrix4d ConstructPoseT(const Eigen::Vector3d& t, const Eigen::Quaterniond& q){
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    res.block<3,3>(0,0) = q.toRotationMatrix();
    res.block<3,1>(0,3) = t;
     return res;
}

Eigen::Vector3d PoseTMulPt(const Eigen::Matrix4d& T, const Eigen::Vector3d& pt){
    return T.block<3,3>(0,0)*pt+T.block<3,1>(0,3);
}
Eigen::Vector3d Qua2Euler(const Eigen::Quaterniond& q){
    Eigen::Vector3d rpy;
    Eigen::Matrix3d q_2_matrix = q.toRotationMatrix();
    //get euler angle according to （Z-Y-X）to get roll,pitch,yaw;
    rpy[1] = asin(-q_2_matrix(2, 0)); // 计算pitch
    if (cos(rpy[1]) != 0) {
        rpy[0] = atan2(q_2_matrix(2, 1), q_2_matrix(2, 2)); // 计算roll
        rpy[2] = atan2(q_2_matrix(1, 0), q_2_matrix(0, 0));  // 计算yaw
    } else {
        rpy[0] = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
        rpy[2] = atan2(-q_2_matrix(0, 1), q_2_matrix(1, 1)); // 计算yaw
    }
    return rpy;
}


class PoseT{
public:
    Eigen::Matrix4d pose;
    PoseT(){}
    PoseT(const Eigen::Vector3d& t, const Eigen::Matrix3d& R){
        pose = Eigen::Matrix4d::Identity();
        pose.block<3,3>(0,0) = R;
        pose.block<3,1>(0,3) = t;
    }
    PoseT(const Eigen::Vector3d& t, const Eigen::Quaterniond& q){
        pose = Eigen::Matrix4d::Identity();
        pose.block<3,3>(0,0) = q.toRotationMatrix();
        pose.block<3,1>(0,3) = t;
    }
    PoseT(const Eigen::Matrix4d &T){
        pose = T;
    }
    Eigen::Vector3d GetXYZ() const{
        return pose.block<3,1>(0,3);
    }
    Eigen::Matrix3d GetR() const{
        return pose.block<3,3>(0,0);
    }
    Eigen::Quaterniond GetQ() const{
        Eigen::Quaterniond q (pose.block<3,3>(0,0));
        return q;
    }
    PoseT operator*(const PoseT& T) const{
        return PoseT(pose * T.pose);
    }

    Eigen::Vector3d operator*(const Eigen::Vector3d& pt) const{
        return pose.block<3,3>(0,0)*pt+pose.block<3,1>(0,3);
    }

    PoseT inverse() const{
        return PoseT(pose.inverse());
    }
    PoseT between(const PoseT& g) const {
        return PoseT(pose.inverse() * g.pose.matrix());
    }
};

#endif
