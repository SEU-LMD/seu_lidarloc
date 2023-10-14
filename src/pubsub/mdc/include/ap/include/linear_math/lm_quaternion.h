/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: linear math数据结构：Quaternion
 */

#ifndef VIZ_LIB_LM_QUATERNION_H
#define VIZ_LIB_LM_QUATERNION_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "lm_vector3.h"

namespace mdc {
namespace visual {
namespace lm {
class LmQuaternion;
LmQuaternion operator*(const LmQuaternion& q1, const LmQuaternion& q2);

double Dot(const LmQuaternion& q1, const LmQuaternion& q2);
double Length(const LmQuaternion& q);
double Angle(const LmQuaternion& q1, const LmQuaternion& q2);
double AngleShortestPath(const LmQuaternion& q1, const LmQuaternion& q2);
LmQuaternion Inverse(const LmQuaternion& q);
LmQuaternion Slerp(const LmQuaternion& q1, const LmQuaternion& q2, const double& t);
// 使用四元数旋转向量
LmVector3  QuatRotate(const LmQuaternion& rotation, const LmVector3& v);
LmQuaternion ShortestArcQuat(const LmVector3& v0, const LmVector3& v1);

// 四元数，用于表示三维空间里的旋转，依赖eigen的Vector4d类型
class LmQuaternion {
public:
    LmQuaternion();
    ~LmQuaternion() = default;
    LmQuaternion(const double &x, const double &y, const double &z, const double &w);
    LmQuaternion(const LmQuaternion &q);
    LmQuaternion& operator=(const LmQuaternion&);
    LmQuaternion(const double& yaw, const double& pitch, const double& roll);
    LmQuaternion(const LmVector3& axis, const double& lmAngle);

    // 打印重载<<
    friend std::ostream &operator<<(std::ostream &out, const LmQuaternion &object)
    {
        return Eigen::operator<<<>(out, object.Qua());
    }

    LmQuaternion operator+(const LmQuaternion& q2) const;
    LmQuaternion operator-(const LmQuaternion& q2) const;
    LmQuaternion operator-() const;
    LmQuaternion operator*(const double& factor) const;
    LmQuaternion operator/(const double& factor) const;
    const LmQuaternion &operator+=(const LmQuaternion &q);
    const LmQuaternion &operator-=(const LmQuaternion &q);
    const LmQuaternion &operator*=(const double &factor);
    LmQuaternion& operator/=(const double& factor);
    // Hamilton product 分配律
    LmQuaternion& operator*=(const LmQuaternion& q);
    bool operator==(const LmQuaternion &other) const;
    bool operator!=(const LmQuaternion &other) const;

    const Eigen::Vector4d &Qua() const;
    // set get 接口
    void SetX(double x);
    void SetY(double y);
    void SetZ(double z);
    void SetW(double w);
    const double &x() const;
    const double &y() const;
    const double &z() const;
    const double &w() const;

    // 设置为other向量的最大值
    void SetMax(const LmQuaternion &other);
    // 设置为other向量的最小值
    void SetMin(const LmQuaternion &other);
    // 设置向量值
    void SetValue(const double &x, const double &y, const double &z, const double &w);
    // 使用轴角度设置旋转向量
    void SetRotation(const LmVector3& axis, const double& lmAngle);
    // 使用欧拉角(YXZ顺序)设置旋转向量: yaw->Y; pitch->X; roll->Z
    void SetEuler(const double& yaw, const double& pitch, const double& roll);
    // 使用欧拉角(XYZ顺序)设置旋转向量: yaw->X; pitch->Y; roll->Z
    void SetRPY(const double& roll, const double& pitch, const double& yaw);
    // 使用欧拉角(ZYX顺序)设置旋转向量: yaw->Z pitch->Y roll->X
    void SetEulerZYX(const double& yaw, const double& pitch, const double& roll);
    // Quaternion的点积
    double Dot(const LmQuaternion& q) const;
    // Quaternion长度的平方，2范数
    double Length2() const;
    // Quaternion长度，2范数
    double Length() const;
    // Quaternion归一化
    LmQuaternion& Normalize();
    // Quaternion归一化
    LmQuaternion Normalized() const;
    // 两个四元数间夹角
    double Angle(const LmQuaternion& q) const;
    // 两个四元数沿最短路径的夹角
    double AngleShortestPath(const LmQuaternion& q) const;
    // 由四元数计算返回0~2pi的角度， z轴
    double GetAngle() const;
    // 由四元数计算返回0~pi的角度沿最短路径， z轴
    double GetAngleShortestPath() const;
    // 返回由四元数代表的向量
    LmVector3 GetAxis() const;
    // 负四元数
    LmQuaternion Inverse() const;
    // 球形线性插值
    LmQuaternion Slerp(const LmQuaternion& q, const double& t) const;
    // 单位四元数
    static const LmQuaternion& GetIdentity()
    {
        static const LmQuaternion identityQuat(0.0, 0.0, 0.0, 1.0);
        return identityQuat;
    }

private:
    Eigen::Vector4d lmQuat_;
};
}
}
}

#endif // VIZ_QUATERNION_H
