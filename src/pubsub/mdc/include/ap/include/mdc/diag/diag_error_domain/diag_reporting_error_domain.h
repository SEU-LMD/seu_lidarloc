/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag reporting error domain
 */
#ifndef MDC_DIAG_REPORTING_ERROR_DOMAIN_H
#define MDC_DIAG_REPORTING_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DiagReportingErrc : mdc::common::ErrorDomain::CodeType {
    INVALID_ARGUMENT = 105,
    GENERIC_ERROR = 107
};
class DiagReportingErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DiagReportingErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DiagReportingErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagReportingErrc::INVALID_ARGUMENT):
                ret = "Invalid Argument";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagReportingErrc::GENERIC_ERROR):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag reporting error";
                break;
        }

        return ret;
    }
private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DiagReportingErrorDomain g_diagReportingErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDiagReportingErrorDomain()
{
    return internal::g_diagReportingErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DiagReportingErrc& code,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(code), GetDiagReportingErrorDomain(),
        supportData);
}
}
}
#endif
