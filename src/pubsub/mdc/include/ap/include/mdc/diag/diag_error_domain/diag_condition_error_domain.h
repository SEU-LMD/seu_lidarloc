/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag condition error domain
 */
#ifndef MDC_DIAG_CONDITION_ERROR_DOMAIN_H
#define MDC_DIAG_CONDITION_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DiagConditionErrc : mdc::common::ErrorDomain::CodeType {
    kGenericError = 107,
    kInvalidArgument = 105
};
class DiagConditionErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DiagConditionErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DiagConditionErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagConditionErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagConditionErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag condition error";
                break;
        }

        return ret;
    }
private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DiagConditionErrorDomain g_diagConditionErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDiagConditionErrorDomain()
{
    return internal::g_diagConditionErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DiagConditionErrc& code,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(code), GetDiagConditionErrorDomain(),
        supportData);
}
}
}
#endif
