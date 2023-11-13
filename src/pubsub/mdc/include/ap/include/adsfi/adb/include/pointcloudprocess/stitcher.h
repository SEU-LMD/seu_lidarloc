/* *
 * FUNCTION: Define Transform Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */
#ifndef HAF_POINTCLOUDPROCESS_STITCHER_H
#define HAF_POINTCLOUDPROCESS_STITCHER_H

#include "core/status.h"
#include "pointcloudprocess/pointcloud_common.h"
namespace Adsfi {
template <typename T> struct HafPointCloudStitcherHandle {
    std::vector<LidarFrame<T>> pointcloudList;      // 输入点云列表
    std::vector<HafMatrix4f> transformMatrix4fList; // 矩阵列表
    LidarFrame<T> stitchedFrame;                    // 拼接后的点云
};

struct HafPointCloudStitcherParams {
    HafTime timestamp;    // 输出点云的时间戳
    uint32_t seq;         // 输出点云的序号
    std::string frameID;  // 输出点云的传感器ID信息
};

template <typename T>
void HafPointCloudStitcherInitialize(HafPointCloudStitcherHandle<T> &stitcherHandle,
    const HafPointCloudStitcherParams &params)
{
    stitcherHandle.pointcloudList.clear();
    stitcherHandle.transformMatrix4fList.clear();
    stitcherHandle.stitchedFrame.pointCloud.clear();
    stitcherHandle.stitchedFrame.timestamp = params.timestamp;
    stitcherHandle.stitchedFrame.seq = params.seq;
    stitcherHandle.stitchedFrame.frameID = params.frameID;
}

/* *
 * 输入的transformMatrix4fList中的每一个变换矩阵都需要满足相关特性：
 * 1、第4行的值为[0 0 0 1]；
 * 2、旋转矩阵R要求是正交矩；
 * 3、旋转矩阵R要求行列式det(R)=1）。
 * */
template <typename T>
HafStatus HafPointCloudStitcherBindInput(HafPointCloudStitcherHandle<T> &stitcherHandle,
    const std::vector<LidarFrame<T>> &pointcloudList, const std::vector<HafMatrix4f> &transformMatrix4fList)
{
    if (pointcloudList.empty()) {
        HAF_LOG_ERROR << "The input pointcloud list is empty.";
        return HAF_ERROR;
    }
    if (pointcloudList.size() != transformMatrix4fList.size()) {
        HAF_LOG_ERROR << "pointcloudList.size() != transformMatrix4fList.size().";
        return HAF_ERROR;
    }
    stitcherHandle.pointcloudList.clear();
    stitcherHandle.transformMatrix4fList.clear();
    stitcherHandle.pointcloudList = pointcloudList;
    stitcherHandle.transformMatrix4fList = transformMatrix4fList;
    return HAF_SUCCESS;
}

template <typename T>
HafStatus HafPointCloudStitcherBindInput(HafPointCloudStitcherHandle<T> &stitcherHandle,
    const std::vector<LidarFrame<T>> &pointcloudList)
{
    if (pointcloudList.empty()) {
        HAF_LOG_ERROR << "The input pointcloud list is empty.";
        return HAF_ERROR;
    }
    stitcherHandle.pointcloudList.clear();
    stitcherHandle.pointcloudList = pointcloudList;
    return HAF_SUCCESS;
}

template <typename T> HafStatus HafPointCloudStitcherProcess(HafPointCloudStitcherHandle<T> &stitcherHandle)
{
    if (stitcherHandle.pointcloudList.empty()) {
        HAF_LOG_ERROR << "The handle pointcloud list is empty.";
        return HAF_ERROR;
    }
    if ((stitcherHandle.pointcloudList.size() != stitcherHandle.transformMatrix4fList.size()) &&
        (!stitcherHandle.transformMatrix4fList.empty())) {
        HAF_LOG_ERROR << "The handle pointcloudList.size() != transformMatrix4fList.size().";
        return HAF_ERROR;
    }
    stitcherHandle.stitchedFrame.pointCloud.clear();
    if (stitcherHandle.transformMatrix4fList.empty()) {
        HAF_LOG_DEBUG <<
            "The handle transformMatrix4f list is empty. Stitch all pointcloud directly, without transformation.";
        for (std::size_t i = 0U; i < stitcherHandle.pointcloudList.size(); ++i) {
            stitcherHandle.stitchedFrame.pointCloud.insert(stitcherHandle.stitchedFrame.pointCloud.end(),
                stitcherHandle.pointcloudList[i].pointCloud.begin(), stitcherHandle.pointcloudList[i].pointCloud.end());
        }
        return HAF_SUCCESS;
    }
    for (std::size_t i = 0U; i < stitcherHandle.pointcloudList.size(); ++i) {
        LidarFrame<T> transformedPC;
        HafStatus status = PointcloudCommon::HafPointCloudCoordinateTransform(stitcherHandle.pointcloudList[i],
            transformedPC, stitcherHandle.transformMatrix4fList[i]);
        if (status != HAF_SUCCESS) {
            HAF_LOG_ERROR << "The " << i << "th time "
                          << "point cloud coordinate transform failed, so stitch failed.";
            return status;
        }
        stitcherHandle.stitchedFrame.pointCloud.insert(stitcherHandle.stitchedFrame.pointCloud.end(),
            transformedPC.pointCloud.begin(), transformedPC.pointCloud.end());
    }
    return HAF_SUCCESS;
}

template <typename T>
void HafPointCloudStitcherBindOutput(const HafPointCloudStitcherHandle<T> &stitcherHandle, LidarFrame<T> &stitchedFrame)
{
    stitchedFrame.pointCloud.clear();
    if (stitcherHandle.stitchedFrame.pointCloud.empty()) {
        HAF_LOG_WARN << "The output pointcloud of handle is empty.";
    }
    stitchedFrame = stitcherHandle.stitchedFrame;
}

template <typename T> void HafPointCloudStitcherRelease(HafPointCloudStitcherHandle<T> &stitcherHandle)
{
    stitcherHandle.pointcloudList.clear();
    stitcherHandle.transformMatrix4fList.clear();
    stitcherHandle.stitchedFrame.pointCloud.clear();
}
}
#endif
