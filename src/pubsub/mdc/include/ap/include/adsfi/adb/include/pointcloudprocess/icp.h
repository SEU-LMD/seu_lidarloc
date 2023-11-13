/*
 * Description: 点云配准ICP算法
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */
#ifndef HAF_POINTCLOUDPROCESS_ICP_H
#define HAF_POINTCLOUDPROCESS_ICP_H

#include <vector>
#include "eigen/haf_eigen.h"
#include "pointcloudprocess/kd_tree.h"

namespace Adsfi {
const float64_t DEFALUT_EPSILION = 1e-3;
const int32_t DEFALUT_ITER_NUM = 50;
const int32_t DEFALUT_SAMPLE_SIZE = 100;
class HafICP {
public:
    HafICP();

    void Init(int32_t itersNum, float64_t eps, int32_t selectPtsNum);

    bool EstimatePose3D(const std::vector<HafEigenVector3d> &targetCloud,
        const std::vector<HafEigenVector3d> &sourceCloud, HafEigenMatrix4d &deltaT);

    bool FindTransformation(const std::vector<HafEigenVector3d> &targetPoints,
        const std::vector<HafEigenVector3d> &sourcePoints, HafEigenMatrix4d &transform);

    bool IfInitialized() const;

private:
    void ConstructMatchedPoints(const std::shared_ptr<KdTree> tree, const std::vector<HafEigenVector3d> &sourcePoints,
        const HafEigenMatrix4d transform, float64_t& squaredErrorCur);
    const float32_t factor = 9.0F;
    const float32_t maxSquareErrorTh = 0.5F;
    float64_t squaredErrorTh = 0.0D;
    int32_t samplePointsNum;
    int32_t maxIterations;
    float64_t epsilon;
    bool initialized_ = false;
    std::vector<HafEigenVector3d> matchedSourcePoints;
    std::vector<HafEigenVector3d> matchedTargetPoints;
};
}


#endif
