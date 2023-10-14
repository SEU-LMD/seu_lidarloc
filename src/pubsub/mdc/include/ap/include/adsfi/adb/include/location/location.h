/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  location.h
 */

#ifndef HAF_LOCATION_LOCATION_H
#define HAF_LOCATION_LOCATION_H

#include "core/types.h"
namespace Adsfi {
    struct HafLocation {
        HafHeader header;
        HafPoseWithCovariance pose;
        HafTwistWithCovariance velocity;
        HafAccelWithCovariance acceleration;
        uint16_t locationState;
        uint8_t odomType = 0U; // 0 inertial, camera, 1 inertial, radar, 2 inertial, lidar, 3 inertial, lidar, camera
    };
}
#endif // HAF_LOCATION_LOCATION_H
