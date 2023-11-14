/* *
 * FUNCTION: Define ICP Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 *              */
#ifndef HAF_POINTCLOUDPROCESS_HAF_ICP_H
#define HAF_POINTCLOUDPROCESS_HAF_ICP_H

#include <vector>
#include "core/status.h"
#include "core/logger.h"
#include "pointcloudprocess/icp.h"
namespace Adsfi {
const int32_t MIN_SAMPLE_SIZE = 4;

struct HafPointCloudICPHandle {
    HafICP handle;
    HafTransformation4f pose; // ICP配准算法输出的转换矩阵
};

struct HafPointCloudICPParams {
    int32_t maximumIterations = DEFALUT_ITER_NUM; // 最大迭代次数
    float32_t euclideanFitnessEpsilon = DEFALUT_EPSILION; // 欧几里德误差的平方和小于此值则终止，单位: 平方米
    int32_t sampleSize = DEFALUT_SAMPLE_SIZE;             // 每次迭代的采样数
};

HafStatus HafPointCloudICPInitialize(HafPointCloudICPHandle &icpHandle, const HafPointCloudICPParams &params)
{
    HafStatus status = HafStatus::HAF_SUCCESS;
    HafPointCloudICPParams validParams;
    if (params.maximumIterations < 0) {
        status = HafStatus::HAF_INVALID_ARGUMENT;
        HAF_LOG_WARN << "MaximumIterations should be positive integer, please try again or use default value : 50";
    } else {
        validParams.maximumIterations = params.maximumIterations;
    }
    if (params.euclideanFitnessEpsilon < 0) {
        status = HafStatus::HAF_INVALID_ARGUMENT;
        HAF_LOG_WARN << "EuclideanFitnessEpsilon should be posity, please try again or use default value : 1e-3";
    } else {
        validParams.euclideanFitnessEpsilon = params.euclideanFitnessEpsilon;
    }
    if (params.sampleSize < MIN_SAMPLE_SIZE) {
        status = HafStatus::HAF_INVALID_ARGUMENT;
        HAF_LOG_WARN <<
            "SampleSize should be an integer greater than or equal to 4, please try again or use default value : 4";
    } else {
        validParams.sampleSize = params.sampleSize;
    }
    if (HafStatus::HAF_SUCCESS != status) {
        return status;
    }
    icpHandle.handle.Init(validParams.maximumIterations, validParams.euclideanFitnessEpsilon, validParams.sampleSize);
    return status;
}

template <typename T>
static void ConvertInputFormat(const std::vector<T> &inCloud, std::vector<HafEigenVector3d> &outCloud)
{
    outCloud.reserve(inCloud.size());
    outCloud.clear();
    for (size_t i = 0; i < inCloud.size(); i++) {
        HafEigenVector3d pt(inCloud[i].x, inCloud[i].y, inCloud[i].z);
        outCloud.emplace_back(pt);
    }
}

template <typename T>
HafStatus HafPointCloudICPProcess(HafPointCloudICPHandle &icpHandle, std::vector<T> &sourcePointCloud,
    const std::vector<T> &targetPointCloud)
{
    if (sourcePointCloud.size() < MIN_SAMPLE_SIZE) {
        HAF_LOG_ERROR << "Source pointcloud size < " << MIN_SAMPLE_SIZE;
        return HAF_ERROR;
    }
    if (targetPointCloud.size() < MIN_SAMPLE_SIZE) {
        HAF_LOG_ERROR << "Target pointcloud size < " << MIN_SAMPLE_SIZE;
        return HAF_ERROR;
    }
    if (!icpHandle.handle.IfInitialized()) {
        HAF_LOG_WARN <<
            "Algorithnm has not been initialized. All parameters ara still default value. maximumIterations: " <<
            DEFALUT_ITER_NUM << ", euclideanFitnessEpsilon : " << DEFALUT_EPSILION << ", sampleSize: " <<
            DEFALUT_SAMPLE_SIZE;
    }
    std::vector<HafEigenVector3d> targetPts;
    std::vector<HafEigenVector3d> sourcePts;
    ConvertInputFormat<T>(targetPointCloud, targetPts);
    ConvertInputFormat<T>(sourcePointCloud, sourcePts);
    HafEigenMatrix4d transformMatrix = HafEigenMatrix4dIdentity();
    bool status = icpHandle.handle.FindTransformation(targetPts, sourcePts, transformMatrix);
    if (status) {
        int32_t dimention = transformMatrix.rows();
        for (int32_t i = 0; i < transformMatrix.rows(); ++i) {
            for (int32_t j = 0; j < transformMatrix.cols(); ++j) {
                icpHandle.pose.array[i * dimention + j] = transformMatrix(i, j);
            }
        }
        return HAF_SUCCESS;
    }
    return HAF_ERROR;
}
}
#endif
