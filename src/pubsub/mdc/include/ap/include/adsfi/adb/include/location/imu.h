/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  IMU.h IMU数据结构体
 */

#ifndef HAF_LOCATION_IMU_H
#define HAF_LOCATION_IMU_H

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafIMU
        功能描述		:  提供IMU数据信息
    ****************************************************************************** */
    struct HafIMU {
        HafHeader imuHeader;
        Point3d angularVelocity;
        Point3d linearAcceleration;
        uint16_t imuStatus;
        float32_t temperature;
    };
}
#endif // HAF_LOCATION_IMU_H
