/* *
 * FUNCTION: Packaging Eigen
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 *     */
#ifndef ADSF_INTERFACES_EIGEN_HAFEIGEN_H
#define ADSF_INTERFACES_EIGEN_HAFEIGEN_H

#include "Eigen/Core"
#include "Eigen/Dense"

namespace Adsfi {
using HafEigenMatrix4f = Eigen::Matrix4f;
using HafEigenMatrix4d = Eigen::Matrix4d;
using HafEigenMatrixXd = Eigen::MatrixXd;
using HafEigenMatrix3d = Eigen::Matrix3d;
using HafEigenVector4f = Eigen::Vector4f;
using HafEigenVector3d = Eigen::Vector3d;
using HafEigenVector4d = Eigen::Vector4d;
using HafEigenVectorXf = Eigen::VectorXf;
using HafEigenVectorXd = Eigen::VectorXd;
using HafEigenMatrix2d = Eigen::Matrix2d;
using HafEigenVector3f = Eigen::Vector3f;
using HafEigenVector2d = Eigen::Vector2d;
using HafEigenQuaterniond = Eigen::Quaterniond;
using HafEigenAngleAxisd = Eigen::AngleAxisd;


template <typename T> using HafDynamicMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
template <typename T> using HafEigenJacobiSVD = Eigen::JacobiSVD<T>;
template <typename T> using HafEigenMap = Eigen::Map<T>;
template <typename T> using HafEigenaligned_allocator = Eigen::aligned_allocator<T>;

enum HafEigenDecompositionOptions {
    HAF_EIGEN_ComputeFullU = Eigen::ComputeFullU,
    HAF_EIGEN_ComputeThinU = Eigen::ComputeThinU,
    HAF_EIGEN_ComputeFullV = Eigen::ComputeFullV,
    HAF_EIGEN_ComputeThinV = Eigen::ComputeThinV,
    HAF_EIGEN_EigenvaluesOnly = Eigen::EigenvaluesOnly,
    HAF_EIGEN_ComputeEigenvectors = Eigen::ComputeEigenvectors,
    HAF_EIGEN_Ax_lBx = Eigen::Ax_lBx,
    HAF_EIGEN_ABx_lx = Eigen::ABx_lx,
    HAF_EIGEN_BAx_lx = Eigen::BAx_lx
};

inline HafEigenMatrix4f HafEigenMatrix4fIdentity()
{
    return Eigen::Matrix4f::Identity();
}

inline HafEigenMatrix4d HafEigenMatrix4dIdentity()
{
    return Eigen::Matrix4d::Identity();
}

inline HafEigenMatrix3d HafEigenMatrix3dZero()
{
    return Eigen::Matrix3d::Zero();
}
}
#endif
