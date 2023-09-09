//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_BASE_TYPE_H
#define SEU_LIDARLOC_BASE_TYPE_H

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/utility.h"
//所有数据的基类

enum DataType {IMU, LIDAR, WHEEL, GNSS_INS,//from sensor
                ODOMETRY, PATH};//common used type

class BaseType{
public:
    double timesamp;
    std::string frame;
    virtual DataType getType() = 0;
};

class CloudType:public BaseType{
    public:
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
        DataType getType(){
            return DataType::LIDAR;
        }
};

class OdometryType:public BaseType{
    public:
        PoseT pose;
    DataType getType(){
        return DataType::ODOMETRY;
    }
};

class GNSSINSType:public BaseType{
    public:
        PoseT pose;
        Eigen::Vector3d imu_angular_v;
        Eigen::Vector3d imu_linear_v;
        Eigen::Matrix<double,6,1> cov;//组合导航设备的置信度
        int gps_status;
        DataType getType(){
             return DataType::GNSS_INS;
        }
};

class PathType:public BaseType{
    public:
        std::vector<PoseT> poses;
        DataType getType(){
            return DataType::PATH;
        }
};

#endif //SEU_LIDARLOC_BASE_TYPE_H
