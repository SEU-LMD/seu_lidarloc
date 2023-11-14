/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag operation cycle error domain
 */
#ifndef MDC_DTC_INFORMATION_ERROR_DOMAIN_H
#define MDC_DTC_INFORMATION_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DtcInformationErrc : mdc::common::ErrorDomain::CodeType {
    INVALID_ARGUMENT = 105,
    GENERIC_ERROR = 107
};
class DtcInformationErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DtcInformationErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DtcInformationErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<mdc::common::ErrorDomain::CodeType>(DtcInformationErrc::INVALID_ARGUMENT):
                ret = "Invalid Argument";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DtcInformationErrc::GENERIC_ERROR):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown dtc information error";
                break;
        }

        return ret;
    }
private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DtcInformationErrorDomain g_dtcInformationErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDtcInformationErrorDomain()
{
    return internal::g_dtcInformationErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DtcInformationErrc& code,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(code), GetDtcInformationErrorDomain(),
        supportData);
}
}
}
#endif
