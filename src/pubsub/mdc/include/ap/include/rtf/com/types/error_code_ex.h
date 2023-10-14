/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Method result definition
 * Create: 2020-12-24
 */
#ifndef RTF_COM_TYPES_ERROR_CODE_EX_H_
#define RTF_COM_TYPES_ERROR_CODE_EX_H_
#include "rtf/com/types/ros_types.h"
namespace rtf {
namespace com {
/**
 * @brief Extended error code, which indicates the error code received from the CM application.
 */
class ErrorCodeEx {
public:
    ErrorCodeEx(const ErrorCodeExDomainType& domain, const ErrorCodeExValueType& value)
        : domain_(domain), value_(value)
    {
    }
    ~ErrorCodeEx() = default;
    ErrorCodeExDomainType Domain() const noexcept
    {
        return domain_;
    }
    ErrorCodeExValueType Value() const noexcept
    {
        return value_;
    }
private:
    ErrorCodeExDomainType domain_;
    ErrorCodeExValueType value_;
};
} // namespace com
} // namespace rtf
#endif
