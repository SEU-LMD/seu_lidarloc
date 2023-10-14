/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag condition error domain
 */
#ifndef ARA_DIAG_CONDITION_ERROR_DOMAIN_H
#define ARA_DIAG_CONDITION_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DiagConditionErrc : ara::core::ErrorDomain::CodeType {
    kInvalidArgument = 105,
    kGenericError = 107
};
class DiagConditionErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DiagConditionErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagConditionErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagConditionErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagConditionErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag condition error";
                break;
        }

        return ret;
    }
    // not to throw exception
    void ThrowAsException(ara::core::ErrorCode const &) const noexcept(false) override {}
private:
    constexpr static ara::core::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DiagConditionErrorDomain g_diagConditionErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDiagConditionErrorDomain()
{
    return internal::g_diagConditionErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagConditionErrc code,
    const ara::core::ErrorDomain::SupportDataType supportdata = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDiagConditionErrorDomain(),
        supportdata);
}
}
}
#endif
