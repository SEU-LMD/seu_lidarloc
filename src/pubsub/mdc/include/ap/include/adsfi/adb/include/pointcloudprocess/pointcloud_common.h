/* *
 * FUNCTION: Define HafPointCloudCommon Function
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */
#ifndef HAF_POINTCLOUDPROCESS_COMMON_H
#define HAF_POINTCLOUDPROCESS_COMMON_H

#include <cstddef>
#include <cstdlib>
#include <cfloat>
#include "core/status.h"
#include "core/types.h"
#include "core/logger.h"
#include "eigen/haf_eigen.h"
namespace Adsfi {
bool IfEqualFloat(const float32_t &floatNum1, const float32_t &floatNum2);
HafEigenMatrix4f HafToEigenMatrix4f(const HafMatrix4f &hafTransformation);
void HafCrossOp(const Point3d &a, const Point3d &b, Point3d &c);
bool HafDivideOp(Point3d &point, const float64_t &denom);
float64_t HafNormOp(const Point3d &point);
float64_t HafDotOp(const Point3d &p1, const Point3d &p2);
float64_t HafDotOp(const Point4d &p1, const Point4d &p2);
bool HafIsZero(const Point4d &point);

namespace PointcloudCommon {
/* ******************************************************************************
函 数 名		:  HafPointCloudCoordinateTransform
功能描述		:  点云坐标变换
输入参数		:  sourceFrame : 原始点云 ; hafTransformation : 变换矩阵
输出参数		:  HafStatus : 状态值 ; transformedPC : 变换后的点云
****************************************************************************** */
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
    if (sourceFrame.pointCloud.empty()) {
        HAF_LOG_ERROR << "The input point cloud is empty.";
        return HAF_ERROR;
    }
    const int32_t index30 = 12;
    const int32_t index31 = 13;
    const int32_t index32 = 14;
    const int32_t index33 = 15;
    if (!(IfEqualFloat(hafTransformation.array[index30], 0.0F) &&
          IfEqualFloat(hafTransformation.array[index31], 0.0F) &&
          IfEqualFloat(hafTransformation.array[index32], 0.0F) &&
          IfEqualFloat(hafTransformation.array[index33], 1.0F))) {
        HAF_LOG_ERROR << "The input transform matrix is wrong.";
        return HAF_ERROR;
    }
    HafEigenMatrix4f transformation = HafToEigenMatrix4f(hafTransformation);
    transformedPC = sourceFrame;
    for (size_t i = 0U; i < sourceFrame.pointCloud.size(); i++) {
        const int32_t axisX = 0;
        const int32_t axisY = 1;
        const int32_t axisZ = 2;
        const auto &point = sourceFrame.pointCloud[i];
        HafEigenVector4f pointVec(point.x, point.y, point.z, 1.0F);
        HafEigenVector4f pointTrans = transformation * pointVec;
        transformedPC.pointCloud[i].x = pointTrans(axisX);
        transformedPC.pointCloud[i].y = pointTrans(axisY);
        transformedPC.pointCloud[i].z = pointTrans(axisZ);
    }
    return HAF_SUCCESS;
}
}
}
#endif
