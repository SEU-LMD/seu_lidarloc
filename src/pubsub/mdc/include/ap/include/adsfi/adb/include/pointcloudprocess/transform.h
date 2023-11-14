/* *
 * FUNCTION: Define Transform Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */
#ifndef HAF_POINTCLOUDPROCESS_TRANSFORM_H
#define HAF_POINTCLOUDPROCESS_TRANSFORM_H

#include "core/status.h"
#include "pointcloudprocess/pointcloud_common.h"
namespace Adsfi {
/* *
 * 输入的变换矩阵hafTransformation需要满足相关特性：
 * 1、第4行的值为[0 0 0 1]；
 * 2、旋转矩阵R要求是正交矩；
 * 3、旋转矩阵R要求行列式det(R)=1）。
 * */
template <typename T>
HafStatus HafPointCloudCoordinateTransform(const LidarFrame<T> &sourceFrame, LidarFrame<T> &transformedPC,
    const HafMatrix4f &hafTransformation)
{
    HafStatus status =
        PointcloudCommon::HafPointCloudCoordinateTransform(sourceFrame, transformedPC, hafTransformation);
    return status;
}
}
#endif
