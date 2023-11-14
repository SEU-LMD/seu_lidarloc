/*
 * Description: 3D KdTree搜索算法
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */
#ifndef HAF_POINTCLOUDPROCESS_KD_TREE_H
#define HAF_POINTCLOUDPROCESS_KD_TREE_H

#include <vector>
#include <memory>
#include "eigen/haf_eigen.h"
#include "core/types.h"

namespace Adsfi {
const int32_t SORT_ON_X = 0;
const int32_t SORT_ON_Y = 1;
const int32_t SORT_ON_Z = 2;
const int32_t DIMENTION = 3;
const float32_t HALF = 0.5F;

class KdTree : public std::enable_shared_from_this<KdTree> {
public:
    explicit KdTree(std::vector<HafEigenVector3d> &pointCloud);
    KdTree(std::vector<HafEigenVector3d> &pointCloud, int32_t start, int32_t end, int32_t sortOn);
    virtual ~KdTree();
    void Build(std::vector<HafEigenVector3d> &pointCloud, int32_t start, int32_t end, int32_t sortOn);

    bool IsLeaf();
    float32_t Split();
    std::shared_ptr<KdTree> GetChild(const HafEigenVector3d &searchPoint);
    float32_t NodeX();
    float32_t NodeY();
    float32_t NodeZ();

    void Search(const HafEigenVector3d &pointIn, HafEigenVector3d &result);
    void RadiusSearch(const HafEigenVector3d &pointIn, float64_t &radius, HafEigenVector3d &result);

private:
    std::shared_ptr<KdTree> leftChild_;
    std::shared_ptr<KdTree> rightChild_;
    HafEigenVector3d node_;
    int32_t sortOn_ = 0;

    void InsertionSort(std::vector<HafEigenVector3d> &pointCloud, int32_t start, int32_t end, int32_t sortOn);
    int32_t GetNextSortOn(int32_t sortOn);
};
}
#endif
