/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: linear math数据结构：Matrix3x3
 */

#ifndef VIZ_LIB_LM_MATRIX3X3_H
#define VIZ_LIB_LM_MATRIX3X3_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "lm_vector3.h"
#include "lm_quaternion.h"

namespace mdc {
namespace visual {
namespace lm {
class LmMatrix3x3;

LmVector3 operator*(const LmMatrix3x3 &m, const LmVector3 &v);
LmVector3 operator*(const LmVector3 &v, const LmMatrix3x3 &m);
LmMatrix3x3 operator*(const LmMatrix3x3 &m1, const LmMatrix3x3 &m2);

// 旋转矩阵，用于一个坐标系中的坐标在另一个坐标系中表示的转换关系，依赖eigen库Matrix3d类型
class LmMatrix3x3 {
public:
    LmMatrix3x3();
    ~LmMatrix3x3() = default;
    LmMatrix3x3(const double &x1, const double &y1, const double &z1,
        const double &x2, const double &y2, const double &z2,
        const double &x3, const double &y3, const double &z3);
    LmMatrix3x3(const LmMatrix3x3 &other);
    explicit LmMatrix3x3(const LmQuaternion &q);
    LmMatrix3x3 &operator=(const LmMatrix3x3 &other);
    bool operator==(const LmMatrix3x3 &other) const;

    // 打印重载<<
    friend std::ostream &operator<<(std::ostream &out, const LmMatrix3x3 &object)
    {
        return Eigen::operator<<<>(out, object.Mat());
    }

    const Eigen::Matrix3d &Mat() const;
    LmVector3 GetColumn(int i) const;
    LmVector3 GetRow(int i) const;
    double &operator()(int row, int col);
    const double &operator()(int row, int col) const;
    LmMatrix3x3 &operator*=(const LmMatrix3x3 &m);
    void SetValue(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2,
        const double &z2, const double &x3, const double &y3, const double &z3);
    // 使用四元数设置旋转矩阵
    void SetRotation(const LmQuaternion &q);
    // 由旋转矩阵获取四元数
    void GetRotation(LmQuaternion &q) const;
    // 使用欧拉角设置旋转矩阵  yaw->Z; pitch->Y; roll->X
    void SetEulerZYX(const double &yaw, const double &pitch, const double &roll);
    void GetEulerZYX(double &yaw, double &pitch, double &roll) const;
    // 使用zyx设置旋转矩阵 yaw->Z pitch->Y roll->X
    void SetEulerYPR(const double &eulerZ, const double &eulerY, const double &eulerX);
    void GetEulerYPR(double &yaw, double &pitch, double &roll) const;
    // 使用rpy 欧拉zyx 设置旋转矩阵 yaw->X; pitch->Y; roll->Z
    void SetRPY(const double &roll, const double &pitch, const double &yaw);
    void GetRPY(double &roll, double &pitch, double &yaw) const;
    // 标量乘法
    LmMatrix3x3 Scaled(const LmVector3 &s) const;
    // 矩阵行列式
    double Determinant() const;
    // 伴随矩阵
    LmMatrix3x3 Adjoint() const;
    // 矩阵绝对值
    LmMatrix3x3 Absolute() const;
    // 矩阵转置
    LmMatrix3x3 Transpose() const;
    // 求逆，要求矩阵的行列式不等于零
    LmMatrix3x3 Inverse() const;
    LmMatrix3x3 TransposeTimes(const LmMatrix3x3 &m) const;
    LmMatrix3x3 TimesTranspose(const LmMatrix3x3 &m) const;
    // 对角化,n阶方阵 A可通过相似变换对角化的充要条件是它具有n个线性无关的特征向量
    void Diagonalize(LmMatrix3x3 &rot);
    // 计算辅助因子 http://en.wikipedia.org/wiki/Cofactor_(linear_algebra)
    double Cofac(uint32_t r1, uint32_t c1, uint32_t r2, uint32_t c2) const;
    // 单位旋转矩阵
    void SetIdentity();
    static const LmMatrix3x3 &GetIdentity()
    {
        // 单位阵
        static const LmMatrix3x3 m(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        return m;
    }
private:
    Eigen::Matrix3d lmMat_;
};
}
}
}

#endif // VIZ_MATRIX3X3_H