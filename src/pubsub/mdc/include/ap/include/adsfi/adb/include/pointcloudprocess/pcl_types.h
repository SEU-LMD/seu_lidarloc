/* *
 * FUNCTION: Define common types for PCL
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *       */
#ifndef HAF_POINTCLOUDPROCESS_PCLTYPES_H
#define HAF_POINTCLOUDPROCESS_PCLTYPES_H
namespace Adsfi {
    enum class HafSACModelType {
        HAF_SACMODEL_PLANE,
        HAF_SACMODEL_LINE,
        HAF_SACMODEL_CIRCLE2D,
        HAF_SACMODEL_CIRCLE3D,
        HAF_SACMODEL_SPHERE,
        HAF_SACMODEL_CYLINDER,
        HAF_SACMODEL_CONE,
        HAF_SACMODEL_TORUS,
        HAF_SACMODEL_PARALLEL_LINE,
        HAF_SACMODEL_PERPENDICULAR_PLANE,
        HAF_SACMODEL_PARALLEL_LINES,
        HAF_SACMODEL_NORMAL_PLANE,
        HAF_SACMODEL_NORMAL_SPHERE,
        HAF_SACMODEL_REGISTRATION,
        HAF_SACMODEL_REGISTRATION_2D,
        HAF_SACMODEL_PARALLEL_PLANE,
        HAF_SACMODEL_NORMAL_PARALLEL_PLANE,
        HAF_SACMODEL_STICK
    };

    enum class HafSACMethodType {
        HAF_SAC_RANSAC = 0,
        HAF_SAC_LMEDS = 1,
        HAF_SAC_MSAC = 2,
        HAF_SAC_RRANSAC = 3,
        HAF_SAC_RMSAC = 4,
        HAF_SAC_MLESAC = 5,
        HAF_SAC_PROSAC = 6,
    };
}
#endif
