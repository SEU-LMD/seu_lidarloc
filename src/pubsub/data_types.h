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

enum DataType {IMU, LIDAR,GNSS, WHEEL, GNSS_INS,//from sensor
                ODOMETRY, PATH,MAP};//common used type

class BaseType{
public:
    double timestamp;
    std::string frame;
    virtual DataType getType() = 0;
};

struct PointXYZIRT {
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t ring;
    double latency;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT ( PointXYZIRT,
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
        pcl::PointCloud<PointXYZIRT> cloud;
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

class DROdometryType:public BaseType{
public:
    PoseT pose;
    DataType getType(){
        return DataType::WHEEL;
    }
};
typedef std::shared_ptr<DROdometryType> DROdometryTypePtr;

class GNSSOdometryType:public BaseType{
public:
    PoseT pose;
    Eigen::Matrix<double,6,1> cov;
    DataType getType(){
        return DataType::GNSS;
    }
};
typedef std::shared_ptr<GNSSOdometryType> GNSSOdometryTypePtr;

class IMUOdometryType:public BaseType{
public:
    PoseT pose;
    DataType getType(){
        return DataType::IMU;
    }
};
typedef std::shared_ptr<IMUOdometryType> IMUOdometryTypePtr;

class GNSSINSType:public BaseType{
    public:
        Eigen::Vector3d lla;
        double roll,pitch,yaw;
        Eigen::Vector3d imu_angular_v;
        Eigen::Vector3d imu_linear_acc;
        Eigen::Matrix<double,6,1> cov;//组合导航设备的置信度
        double velocity;
        string gps_status;
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

class IMURawWheelData{
public:
    double timestamp;
    Eigen::Vector3d imu_angular_v;
    Eigen::Vector3d imu_linear_acc;
    Eigen::Vector3d position;
    double velocity;
};
typedef std::shared_ptr<IMURawWheelData> IMURawWheelDataPtr;

class StateData{
public:
    double timestamp;
    Eigen::Vector3d p_wb_;
    Eigen::Vector3d v_w_;
    Eigen::Matrix3d Rwb_;
    Eigen::Vector3d acc_bias_;
    Eigen::Vector3d gyr_bias_;
};
typedef std::shared_ptr<StateData> StateDataPtr;

//used to commnicate with other thread
class CloudInfo{
public:
    int frame_id;
    double timestamp;
    PoseT pose;
    PoseT DRPose;

//    std::vector<int> label;
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
    PoseT DRPose;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
};
typedef std::shared_ptr<CloudFeature> CloudFeaturePtr;

class PriorMap:public BaseType{
public:
    pcl::KdTreeFLANN<PointType>::Ptr PriorSurfMapKDTree;
    pcl::KdTreeFLANN<PointType>::Ptr PriorCornerMapKDTree;
    pcl::PointCloud<PointType>::Ptr PriorSurfMap;
    pcl::PointCloud<PointType>::Ptr PriorCornerMap;
    DataType getType(){
        return DataType::MAP;
    }
};
typedef std::shared_ptr<PriorMap> PriorMapPtr;

#endif //SEU_LIDARLOC_BASE_TYPE_H
