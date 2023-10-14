/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  selectPoint.h 选择终点
 */


#ifndef LOCATION_SELECT_POINT_H
#define LOCATION_SELECT_POINT_H

#include "core/types.h"
namespace Adsfi {
    struct HafSelectPoint {
        HafHeader header;  // 时间戳
        Point3d point;     // point三维点
    };
}
#endif // LOCATION_SELECT_POINT_H
