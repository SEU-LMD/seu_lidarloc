/* *
 * FUNCTION: Define RangeImage Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */
#ifndef HAF_POINTCLOUDPROCESS_RANGE_IMAGE_H
#define HAF_POINTCLOUDPROCESS_RANGE_IMAGE_H

#include <cmath>
#include "core/status.h"
#include "core/types.h"
#include "core/logger.h"
namespace Adsfi {
    // Need Lidar-Ring
    struct HafPointCloudRangeImageHandle {
        LidarFrame<PointXYZIR> pointCloudFrame; // 输入点云
        RangeImageFrame rangeImage;        // 输出的深度图
    };

    struct HafPointCloudRangeImageParams {
        int32_t horizontalResolution{}; // 横向像素点数量，等于点云视场角范围内，单根线完全反射时的lidar点总数，即前视图的宽
        int32_t verticalResolution{};   // 纵向像素点数量，等于该lidar点云的总线数，从上往下递增，从0开始，即前视图的高
        float32_t startAngle{};         // 等于需要关注点云范围的顺时针方向边界对应的角度值（在lidar传感器坐标系下，右手坐标系）
        float32_t fovAngle{};           // 等于需要关注点云范围的视场角角度（在lidar传感器坐标系下）
    };

    HafStatus HafPointCloudRangeImageInitialize(HafPointCloudRangeImageHandle &rangeImageHandle,
        const HafPointCloudRangeImageParams &params);

    HafStatus HafPointCloudRangeImageBindInput(HafPointCloudRangeImageHandle &rangeImageHandle,
        const LidarFrame<PointXYZIR> &pointCloudFrame);

    HafStatus HafPointCloudRangeImageProcess(HafPointCloudRangeImageHandle &rangeImageHandle);

    HafStatus HafPointCloudRangeImageBindOutput(const HafPointCloudRangeImageHandle &rangeImageHandle,
        RangeImageFrame &outputImage);

    void HafPointCloudRangeImageRelease(HafPointCloudRangeImageHandle &rangeImageHandle);
}
#endif
