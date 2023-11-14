/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Index the error class include domain & codetype
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_API_INTERNAL_INTERNAL_TYPES_H
#define VRTF_VCC_API_INTERNAL_INTERNAL_TYPES_H
#include "vrtf/vcc/api/types.h"

namespace vrtf {
namespace vcc {
namespace api {
namespace types {
namespace internal {
class ErrorIndex {
public:
    ErrorIndex(vrtf::core::ErrorDomain::IdType domain, vrtf::core::ErrorDomain::CodeType codeType)
        : domain_(domain), codeType_(codeType){}
    ~ErrorIndex() = default;
    bool operator<(const ErrorIndex& other) const
    {
        if (domain_ < other.GetDomain()) {
            return true;
        }
        if (domain_ == other.GetDomain() && codeType_ < other.GetCode()) {
            return true;
        }
        return false;
    }
    vrtf::core::ErrorDomain::IdType GetDomain() const
    {
        return domain_;
    }
    vrtf::core::ErrorDomain::CodeType GetCode() const
    {
        return codeType_;
    }

private:
    vrtf::core::ErrorDomain::IdType domain_;
    vrtf::core::ErrorDomain::CodeType codeType_;
};
}
}
}
}
}

#endif
