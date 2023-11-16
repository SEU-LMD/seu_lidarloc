/*
 * @FilePath: sa_types.h
 * @Author: yujie
 * @Date: 2023-05-15 15:08:41
 * @LastEditors: yujie yujie@shineauto.com.cn
 * @LastEditTime: 2023-10-27 14:30:00
 * Copyright: 2023 ShineAuto CO.,LTD. All Rights Reserved.
 * @Descripttion:
 */
#ifndef SA_TYPES_H
#define SA_TYPES_H

#include <vector>
#include "core/basic_types.h"
#include "control/body_report.h"

namespace Adsfi {
struct HafCanTxFrame
{
    uint32_t seq;
    HafTime timestamp;
    uint32_t canId;
    uint32_t canIdType;  // 0 can标准帧，1 can扩展帧， 2 canfd标准帧，3 canfd扩展帧
    std::vector<uint8_t> data;
};

struct HafCanRxBaseFrame
{
    HafTime timestamp;
    uint32_t canId;
    std::vector<uint8_t> data;
};

struct HafCanRxFrame
{
    uint32_t seq;
    std::vector<HafCanRxBaseFrame> elements;
};

struct LidarFrameV2
{
    HafTime timestamp;
    uint32_t seq;
    std::string frameId;
    std::vector<uint8_t> rawData;
};

struct PointXYZIRTV2
{
    float x;
    float y;
    float z;
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
};

struct WheelSpeed
{
    float ESCWhlRRSpd;
    float ESCWhlRLSpd;
    float ESCWhlFRSpd;
    float ESCWhlFLSpd;
};

struct HafLocationV2
{
    HafHeader header;
    Point3D<float64_t> imuAngular;
    Point3D<float64_t> imuAccel;
    Point3D<float64_t> position;
    Point3D<float64_t> positionSigma;
    float64_t velocity;
    float64_t velocitySigma;
    Point3D<float64_t> accel;
    Point3D<float64_t> accelSigma;
    Point3D<float64_t> pose;
    Point3D<float64_t> poseSigma;
    Point3D<float64_t> angular;
    WheelSpeed whlSpd;
    uint16_t locationState;
};

}  // namespace Adsfi

#endif
