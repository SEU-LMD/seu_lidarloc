/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: linear math数据结构：Vector3
 */

#ifndef VIZ_LIB_LM_VECTOR3_H
#define VIZ_LIB_LM_VECTOR3_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace mdc {
namespace visual {
namespace lm {
class LmVector3;
LmVector3 operator+(const LmVector3 &vecA, const LmVector3 &vecB);
LmVector3 operator*(const LmVector3 &vecA, const LmVector3 &vecB);
LmVector3 operator-(const LmVector3 &vecA, const LmVector3 &vecB);
LmVector3 operator-(const LmVector3 &v);
LmVector3 operator*(const LmVector3 &v, const double &s);
LmVector3 operator*(const double &s, const LmVector3 &v);
LmVector3 operator/(const LmVector3 &v, const double &s);
LmVector3 operator/(const LmVector3 &vecA, const LmVector3 &vecB);
double TfDot(const LmVector3 &vecA, const LmVector3 &vecB);
double TfDistance2(const LmVector3 &vecA, const LmVector3 &vecB);
double TfDistance(const LmVector3 &vecA, const LmVector3 &vecB);
double TfAngle(const LmVector3 &vecA, const LmVector3 &vecB);
LmVector3 TfCross(const LmVector3 &vecA, const LmVector3 &vecB);
double TfTriple(const LmVector3 &vecA, const LmVector3 &vecB, const LmVector3 &vecC);
LmVector3 Lerp(const LmVector3 &vecA, const LmVector3 &vecB, const double &t);
// p: n投影至平面的单位向量，n.z() > sqrt(2)/2 => yz平面，n.z() <= sqrt(2)/2 =>xy平面; q: n x p
void TfPlaneSpace1(const LmVector3 &nVec, LmVector3 &pVec, LmVector3 &qVec);

// 向量，用于表示tf中的平移变换，依赖eigen中的Vector3d
class LmVector3 {
public:
    LmVector3();
    ~LmVector3() = default;
    LmVector3(const double &x, const double &y, const double &z);
    LmVector3(const LmVector3 &v);
    LmVector3& operator=(const LmVector3&);

    const LmVector3 &operator+=(const LmVector3 &v);
    const LmVector3 &operator-=(const LmVector3 &v);
    const LmVector3 &operator*=(const double &factor);
    const LmVector3 &operator/=(const double &factor);
    bool operator==(const LmVector3 &other) const;
    bool operator!=(const LmVector3 &other) const;

    // 打印重载<<
    friend std::ostream &operator<<(std::ostream &out, const LmVector3 &object)
    {
        return Eigen::operator<<<>(out, object.Vec());
    }

    const Eigen::Vector3d &Vec() const;
    // 设置xyz向量值
    void SetX(double x);
    void SetY(double y);
    void SetZ(double z);
    // 获取xyz向量值
    const double &x() const;
    const double &y() const;
    const double &z() const;
    // 向量的点积
    double Dot(const LmVector3 &v) const;
    // 向量长度的平方，2范数
    double Length2() const;
    // 向量长度，2范数
    double Length() const;
    // 向量的欧式距离的平方
    double Distance2(const LmVector3 &v) const;
    // 向量的欧式距离
    double Distance(const LmVector3 &v) const;
    // 向量归一化
    LmVector3 Normalize();
    // 向量归一化，返回临时值
    LmVector3 Normalized() const;
    // 旋转向量. wAxis：单位向量，要旋转的轴；angle：旋转的角度
    LmVector3 Rotate(const LmVector3 &wAxis, const double lmAngle) const;
    // 向量间的夹角
    double Angle(const LmVector3 &v) const;
    // 绝对值
    LmVector3 Absolute() const;
    // 向量叉积
    LmVector3 Cross(const LmVector3 &v) const;
    // 3个向量混合积
    double Triple(const LmVector3 &vecA, const LmVector3 &vecB) const;
    // 返回最小值所在位置
    uint32_t MinAxis() const;
    // 返回最大值所在位置
    uint32_t MaxAxis() const;
    // 最远轴，绝对值下最小值所在位置
    uint32_t FurthestAxis();
    // 最近轴，绝对值下最大值所在位置
    uint32_t ClosestAxis();
    // 设置向量插值，该向量为vecA~vecB之间的值
    void SetInterpolate3(const LmVector3 &vecA, const LmVector3 &vecB, double rt);
    // 向量间的线性插值，t=0=>this ,t=1 => vecA
    LmVector3 Lerp(const LmVector3 &v, const double &t) const;
    // 由this生成反对称矩阵  vecA vecB vecC构成3x3矩阵
    void GetSkewSymmetricMatrix(LmVector3 &vecA, LmVector3 &vecB, LmVector3 &vecC) const;
    // 设置为other向量的最大值
    void SetMax(const LmVector3 &other);
    // 设置为other向量的最小值
    void SetMin(const LmVector3 &other);
    // 设置向量值
    void SetValue(const double &x, const double &y, const double &z);
    // 设置为0
    void SetZero();
    // 是否为0，模糊0(double取不到0)
    bool IsZero() const;
private:
    Eigen::Vector3d lmVect_;
};
}
}
}

#endif // VIZ_VECTOR3_H