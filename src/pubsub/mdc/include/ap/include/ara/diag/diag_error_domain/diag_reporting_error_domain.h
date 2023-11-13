/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag reporting error domain
 */
#ifndef ARA_DIAG_REPORTING_ERROR_DOMAIN_H
#define ARA_DIAG_REPORTING_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DiagReportingErrc : ara::core::ErrorDomain::CodeType {
    kInvalidArgument = 105,
    kGenericError = 107
};
class DiagReportingErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DiagReportingErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagReportingErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagReportingErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagReportingErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag reporting error";
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
constexpr DiagReportingErrorDomain g_diagReportingErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDiagReportingErrorDomain()
{
    return internal::g_diagReportingErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagReportingErrc code,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDiagReportingErrorDomain(),
        supportData);
}
}
}
#endif
