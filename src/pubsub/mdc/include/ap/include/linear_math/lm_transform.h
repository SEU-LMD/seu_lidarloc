/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: linear math数据结构：Transform
 */

#ifndef VIZ_LIB_LM_TRANSFORM_H
#define VIZ_LIB_LM_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "lm_vector3.h"
#include "lm_quaternion.h"
#include "lm_matrix3x3.h"

namespace mdc {
namespace visual {
namespace lm {
class LmTransform {
public:
    LmTransform() : basis_(), origin_() {}
    ~LmTransform() = default;
    explicit LmTransform(const LmQuaternion &q, const LmVector3 &v = LmVector3(0.0, 0.0, 0.0)) : basis_(q), origin_(v)
    {}
    explicit LmTransform(const LmMatrix3x3 &m, const LmVector3 &v = LmVector3(0.0, 0.0, 0.0)) : basis_(m), origin_(v) {}
    LmTransform(const LmTransform &other);

    LmTransform &operator=(const LmTransform &other);
    LmVector3 operator()(const LmVector3 &x) const;
    LmVector3 operator*(const LmVector3 &x) const;
    LmQuaternion operator*(const LmQuaternion &q) const;
    LmTransform operator*(const LmTransform &t) const;
    LmTransform &operator*=(const LmTransform &t);
    // 打印重载<<
    friend std::ostream &operator<<(std::ostream &out, const LmTransform &object)
    {
        return out << object.GetBasis() << std::endl << object.GetOrigin();
    }

    void Mult(const LmTransform &t1, const LmTransform &t2);
    const LmMatrix3x3 &GetBasis() const;
    const LmVector3 &GetOrigin() const;
    LmQuaternion GetRotation() const;
    void SetOrigin(const LmVector3 &lmOrigin);
    void SetBasis(const LmMatrix3x3 &lmBasis);
    void SetRotation(const LmQuaternion &q);
    void SetIdentity();
    LmTransform Inverse() const;
    LmVector3 InvXform(const LmVector3 &inVec) const;
    // 这个变换的逆乘另一个变换
    LmTransform InverseTimes(const LmTransform &t) const;
    static const LmTransform &GetIdentity()
    {
        static const LmTransform t(LmMatrix3x3::GetIdentity());
        return t;
    }
private:
    LmMatrix3x3 basis_;
    LmVector3 origin_;
};
}
}
}
#endif // VIZ_TRANSFORM_H