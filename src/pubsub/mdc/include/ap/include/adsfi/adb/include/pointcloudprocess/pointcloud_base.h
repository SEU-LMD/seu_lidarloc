/* *
 * FUNCTION: Define pointCloudBase
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */
#ifndef HAF_POINTCLOUDPROCESS_POINT_CLOUD_BASE_H
#define HAF_POINTCLOUDPROCESS_POINT_CLOUD_BASE_H

#include <vector>
#include "core/status.h"
#include "core/types.h"
#include "core/logger.h"
namespace Adsfi {
    template <typename T> void HafPointCloudCreate(LidarFrame<T> &pointCloudFrame)
    {
        pointCloudFrame.frameID = "";
        pointCloudFrame.seq = 0U;
        pointCloudFrame.timestamp.sec = 0U;
        pointCloudFrame.timestamp.nsec = 0U;
    }

    template <typename T> void HafPointCloudDestroy(LidarFrame<T> &pointCloudFrame)
    {
        pointCloudFrame.pointCloud.clear();
    }
}
#endif
