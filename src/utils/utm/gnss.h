//
// Created by xiang on 2022/1/4.
//

#ifndef SLAM_IN_AUTO_DRIVING_GNSS_H
#define SLAM_IN_AUTO_DRIVING_GNSS_H

#include "eigen_types.h"


namespace sad {

/// GNSS状态位信息
/// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

/// UTM 坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone, const Vec2d& xy = Vec2d::Zero(), bool north = true)
        : zone_(zone), xy_(xy), north_(north) {}

    int zone_ = 0;              // utm 区域
    Vec2d xy_ = Vec2d::Zero();  // utm xy
    double z_ = 0;              // z 高度（直接来自于gps）
    bool north_ = true;         // 是否在北半球
};

}  // namespace sad

//using GNSSPtr = std::shared_ptr<sad::GNSS>;

#endif  // SLAM_IN_AUTO_DRIVING_GNSS_H
