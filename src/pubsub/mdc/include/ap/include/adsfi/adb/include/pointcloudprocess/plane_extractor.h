/* *
 * FUNCTION: Define HafPlaneExtractor Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *         */
#ifndef HAF_POINTCLOUDPROCESS_PCL_HAFPLANEEXTRACTOR_H
#define HAF_POINTCLOUDPROCESS_PCL_HAFPLANEEXTRACTOR_H

#include <vector>
#include "core/status.h"
#include "core/types.h"
#include "core/logger.h"
#include "pointcloudprocess/pcl_types.h"
#include "pointcloudprocess/sac_segmentation.h"
namespace Adsfi {
    const int32_t minPointSize_ = 3;

    struct HafPointCloudPlaneExtractorHandle {
        HafSAC handle;
    };

    HafStatus HafPointCloudPlaneExtractorInitialize(HafPointCloudPlaneExtractorHandle &planeExtractor,
        const float64_t distanceThreshold);

    HafStatus HafPointCloudPlaneExtractorConfig(HafPointCloudPlaneExtractorHandle &planeExtractor,
        const size_t numIterations, const size_t sampleSize);

    template <typename T>
    HafStatus HafPointCloudPlaneExtractorBindInput(HafPointCloudPlaneExtractorHandle &planeExtractor,
        const std::vector<T> &points)
    {
        if (points.size() < minPointSize_) {
            HAF_LOG_ERROR << "The input points' size is less than minimum required size: " << minPointSize_;
            return HafStatus::HAF_INVALID_ARGUMENT;
        } else {
            planeExtractor.handle.SetInputCloud(points);
            return HafStatus::HAF_SUCCESS;
        }
    }

    HafStatus HafPointCloudPlaneExtractorProcess(HafPointCloudPlaneExtractorHandle &planeExtractor,
        std::vector<size_t> &indices, std::vector<float32_t> &modelParams);
}
#endif
