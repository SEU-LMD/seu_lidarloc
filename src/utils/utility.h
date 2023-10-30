#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE
#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "opencv2/opencv.hpp"   // for opencv4
//#include <opencv/cv.h>
#include "Eigen/Dense"


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

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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


sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = SensorConfig::extrinsicRot * acc;//extRot = Eigen::Matrix3d
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = SensorConfig::extrinsicRot  * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                imu_in.orientation.z);
    Eigen::Quaterniond q_final;
    // if (Config::imuType == 0) {
    q_final = SensorConfig::extrinsicQRPY;//extQRPY = Eigen::Quaterniond
    // } else if (imuType == 1)
    //     q_final = q_from * extQRPY;
    // else
    //     std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

    q_final.normalize();
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(
            q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() +
            q_final.w() * q_final.w())
        < 0.1) {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}


template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud,
                                      ros::Time thisStamp,
                                      std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg) {
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

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

//void GetPose(const Eigen::Matrix4d& T, ){
//
//}

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
    /**
     * pose.inverse() * g
     * @param g
     * @return
     */
    PoseT between(const PoseT& g) const {
        return PoseT(pose.inverse() * g.pose.matrix());
    }
    /**
     * Linear Interpolation,
     * requirement: t2 > t1
     * @param g
     * @param interpolation_t1 pose time
     * @param interpolation_t2 g time
     * @return
     */
    PoseT Linear_interpolation(const PoseT& g, const double& interpolation_t1, const double& interpolation_t2){
        PoseT result_output;
        double interpolation_alpha(interpolation_t2-interpolation_t1);
        //t
        Eigen::Vector3d result_output_t;
        result_output_t = pose.block<3,1>(0,3)+
                        interpolation_alpha * (g.GetXYZ() - pose.block<3,1>(0,3));
        //R
        Eigen::Quaterniond result_output_t_q;//旋转
        result_output_t_q = Eigen::Quaterniond(pose.block<3,3>(0,0)).slerp(interpolation_alpha,g.GetQ());

        //插值位姿态矩阵
        return PoseT (result_output_t , result_output_t_q);
    }
};

#endif
