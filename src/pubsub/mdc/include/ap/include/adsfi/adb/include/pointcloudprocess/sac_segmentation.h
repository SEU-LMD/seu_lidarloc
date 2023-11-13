/* *
 * FUNCTION: Define HafSACSegmentation Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *                                     */
#ifndef HAF_POINTCLOUDPROCESS_SACSEGMENTATION_H
#define HAF_POINTCLOUDPROCESS_SACSEGMENTATION_H

#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <random>
#include <unordered_set>
#include <memory>
#include <tuple>
#include <vector>
#include "core/types.h"
#include "pointcloudprocess/pcl_types.h"
#include "pointcloudprocess/pointcloud_common.h"
namespace Adsfi {
class RANSACParam {
public:
    RANSACParam() : fitness_(0), inlierRmse_(0) {}
    ~RANSACParam() {}

public:
    float64_t fitness_;
    float64_t inlierRmse_;
};

class HafSAC {
public:
    HafSAC() = default;
    // 设置点到平面距离阈值
    void SetDistanceThreshold(float64_t threshold);

    // 获取点到平面距离阈值
    float64_t GetDistanceThreshold() const;

    // 输入原始点云
    template <typename T> void SetInputCloud(const std::vector<T> &points)
    {
        inputPoints_.clear();
        for (const T &point : points) {
            Point3d tmp;
            tmp.x = point.x;
            tmp.y = point.y;
            tmp.z = point.z;
            inputPoints_.push_back(tmp);
        }
    }

    // 获得原始点云大小
    int32_t GetInputCloudSize() const
    {
        return inputPoints_.size();
    }
    // 执行分割算法
    HafStatus Segment(std::vector<size_t> &indices, std::vector<float32_t> &modelParams);

    void SetNumIterations(size_t iterations);
    size_t GetNumIterations();

    void SetSampleSize(size_t sampleSize);
    size_t GetSampleSize();

private:
    std::tuple<std::vector<float32_t>, std::vector<size_t>> SegmentPlane();
    Point4d ComputePlane(const std::vector<size_t> &inliers);
    RANSACParam EvaluateRANSACBasedOnDistance(const std::vector<Point3d> &points, const Point4d &planeModel,
        std::vector<size_t> &inliers);
    void RandomSample(std::vector<size_t> &indices, std::vector<size_t> &inliers, std::mt19937 &rng);

    Point4d GetFittingPlane(const std::vector<Point3d> &points, const std::vector<size_t> &inliers);
    std::vector<Adsfi::Point3d> inputPoints_;
    float64_t distanceThreshold_ = 1e-2;
    size_t numIterations_ = 10;
    size_t minSampleSize_ = 3;
};
}
#endif
