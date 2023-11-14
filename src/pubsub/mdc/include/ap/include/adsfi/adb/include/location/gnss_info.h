/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  GNSSInfo.h GPS信息结构体
 */

#ifndef HAF_LOCATION_GNSS_INFO_H
#define HAF_LOCATION_GNSS_INFO_H

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
    结构体 : HafGnssInfo
    功能描述 : GNSS原始定位信息结构体
    ****************************************************************************** */
    struct HafGnssInfo {
        /* data */
        HafHeader header;
        float64_t latitude;                    // 纬度 Unit: deg
        float64_t longitude;                   // 经度 Unit: deg
        float64_t elevation;                   // 海拔高度 Unit: meter
        Point3d utmPosition;                // UTM坐标位置
        int32_t utmZoneNum;                 // UTM区号
        uint8_t utmZoneChar;                // UTM区号
        Point3d attitude;                   // 三轴姿态(roll, ptich, yaw), Unit: rad
        Point3d sdPosition;                 // 位置标准差Unit: meter
        Point3d sdVelocity;                  // 速度标准差Unit: meter
        Point3d sdAttitude;                 // 姿态标准差 Unit: rad
        float64_t second;                      // 时间
        int32_t satUseNum;                  // 使用卫星数
        int32_t satInViewNum;               // 可见卫星数
        uint16_t solutionStatus;             // 求解状态
        uint16_t positionType;               // 定位状态
        Point3d linearVelocity;
        Point3d attitudeDual;               // 双天线姿态
        Point3d sdAngleDual;                // 双天线角度标准差
        float64_t baseLineLengthDual;           // 双天线距离
        int32_t solutionStatusDual;         // 双天线求解状态
        int32_t positionTypeDual;           // 双天线定位状态
        int32_t solutionSourceDual;         // 双天线求解源
        float64_t cep68;
        float64_t cep95;
        float32_t pDop;
        float32_t hDop;
        float32_t vDop;
    };


    /* ******************************************************************************
    结构体 : HafInsInfo
    功能描述 : Ins导远定位信息结构体
    ****************************************************************************** */
    struct HafInsInfo {
        /* data */
        HafHeader header;
        float64_t latitude;                    // 纬度 Unit: deg
        float64_t longitude;                   // 经度 Unit: deg
        float64_t elevation;                   // 海拔高度 Unit: meter
        Point3d utmPosition;                // UTM坐标位置
        int32_t utmZoneNumber;              // UTM区号
        uint8_t utmZoneChar;               // UTM区号
        Point3d attitude;                   // 三轴姿态(roll, ptich, yaw), Unit: rad
        Point3d linearVelocity;
        Point3d sdPosition;                 // 位置标准差Unit: meter
        Point3d sdAttitude;                 // 速度标准差Unit: meter
        Point3d sdVelocity;                 // 姿态标准差 Unit: rad
        float64_t cep68;
        float64_t cep95;
        float64_t second;
        int32_t satUseNum;                  // 使用卫星数
        int32_t satInViewNum;               // 可见卫星数
        uint16_t solutionStatus;             // 求解状态
        uint16_t positionType;               // 定位状态
        float32_t pDop;
        float32_t hDop;
        float32_t vDop;
        Point3d attitudeDual;               // 双天线姿态
        Point3d sdAngleDual;                // 双天线角度标准差
        float64_t baseLineLengthDual;           // 双天线距离
        int32_t solutionStatusDual;         // 双天线求解状态
        int32_t positionTypeDual;           // 双天线定位状态
        int32_t solutionSourceDual;         // 双天线求解源
        uint32_t aoc;
        uint32_t rtkBaseline;
        Point3d angularVelocity;
        Point3d acceleration;
    };
}
#endif  // HAF_LOCATION_GNSS_INFO_H
