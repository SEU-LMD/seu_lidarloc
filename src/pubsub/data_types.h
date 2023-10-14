//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_BASE_TYPE_H
#define SEU_LIDARLOC_BASE_TYPE_H

#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "utils/utility.h"
//所有数据的基类

enum DataType {IMU, LIDAR, WHEEL, GNSS_INS,//from sensor
                ODOMETRY, PATH};//common used type

class BaseType{
public:
    double timestamp;
    std::string frame;
    virtual DataType getType() = 0;
};

//from autoshine
struct PointXYZIRTSEU {
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t ring;
    double latency;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT ( PointXYZIRTSEU,
                                    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                    (uint16_t, ring, ring)(double, latency, latency))

struct PointXYZICOLRANGE {
    PCL_ADD_POINT4D
    uint8_t intensity;
    float range;
    uint16_t col;
    uint8_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT ( PointXYZICOLRANGE,
                                    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                            (uint16_t, range, range)(uint16_t, col, col)(uint8_t, label, label))

//add by lsy
struct PointXYZIL {
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint8_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT ( PointXYZIL,
                                    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                            (uint8_t, label, label))

class CloudTypeXYZIRT: public BaseType{
    public:
        pcl::PointCloud<PointXYZIRTSEU> cloud;
        DataType getType(){
            return DataType::LIDAR;
        }
};
typedef std::shared_ptr<CloudTypeXYZIRT> CloudTypeXYZIRTPtr;

class CloudTypeXYZI: public BaseType{
public:
    pcl::PointCloud<pcl::PointXYZI> cloud;
    DataType getType(){
        return DataType::LIDAR;
    }
};
typedef std::shared_ptr<CloudTypeXYZI> CloudTypeXYZIPtr;

class CloudTypeXYZICOLRANGE: public BaseType{
public:
    pcl::PointCloud<PointXYZICOLRANGE> cloud;
    DataType getType(){
        return DataType::LIDAR;
    }
};
typedef std::shared_ptr<CloudTypeXYZICOLRANGE> CloudTypeXYZICOLRANGEPtr;

class OdometryType:public BaseType{
    public:
        PoseT pose;
        DataType getType(){
            return DataType::ODOMETRY;
        }
};
typedef std::shared_ptr<OdometryType> OdometryTypePtr;

class
GNSSINSType:public BaseType{
    public:
        Eigen::Vector3d lla;//经为高
        double roll,pitch,yaw;//车体系
        Eigen::Vector3d imu_angular_v;//imu原始
        Eigen::Vector3d imu_linear_acc;//imu原始
        Eigen::Matrix<double,6,1> cov;//组合导航设备的置信度
        string  gps_status;
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

class IMURawData{
public:
    double timestamp;
    Eigen::Vector3d imu_angular_v;
    Eigen::Vector3d imu_linear_acc;
    Eigen::Quaterniond orientation;
};
typedef std::shared_ptr<IMURawData> IMURawDataPtr;

//used to commnicate with other thread
class CloudInfo{
public:
    int frame_id;
    double timestamp;
    PoseT pose;
    std::vector<int> startRingIndex;
    std::vector<int> endRingIndex;

    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ptr;
};
typedef std::shared_ptr<CloudInfo> CloudInfoPtr;

class CloudFeature{
public:
    int frame_id;
    double timestamp;
    PoseT pose;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
};
typedef std::shared_ptr<CloudFeature> CloudFeaturePtr;



#endif //SEU_LIDARLOC_BASE_TYPE_H
