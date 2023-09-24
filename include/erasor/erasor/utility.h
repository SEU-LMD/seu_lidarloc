//
// Created by wxy on 23-9-20.
//

#ifndef ERASOR_UTILITY_H
#define ERASOR_UTILITY_H

#endif //ERASOR_UTILITY_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
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

class PoseT{
public:
    Eigen::Matrix4f pose;
    PoseT(){}
    PoseT(const Eigen::Vector3f& t, const Eigen::Matrix3f& R){
        pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = R;
        pose.block<3,1>(0,3) = t;
    }
    PoseT(const Eigen::Vector3f& t, const Eigen::Quaternionf& q){
        pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.toRotationMatrix();
        pose.block<3,1>(0,3) = t;
    }
    PoseT(const Eigen::Matrix4f &T){
        pose = T;
    }
    Eigen::Vector3f GetT() const{
        return pose.block<3,1>(0,3);
    }
    Eigen::Matrix3f GetR() const{
        return pose.block<3,3>(0,0);
    }
    Eigen::Quaternionf GetQ() const{
        Eigen::Quaternionf q (pose.block<3,3>(0,0));
        return q;
    }
    PoseT operator*(const PoseT& T) const{
        return PoseT(pose * T.pose);
    }

    Eigen::Vector3f operator*(const Eigen::Vector3f& pt) const{
        return pose.block<3,3>(0,0)*pt+pose.block<3,1>(0,3);
    }

    PoseT inverse() const{
        return PoseT(pose.inverse());
    }
};