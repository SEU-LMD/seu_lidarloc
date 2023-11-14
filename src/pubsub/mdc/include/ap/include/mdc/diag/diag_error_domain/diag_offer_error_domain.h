/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag offer error domain
 */
#ifndef MDC_DIAG_OFFER_ERROR_DOMAIN_H
#define MDC_DIAG_OFFER_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DiagOfferErrc : mdc::common::ErrorDomain::CodeType {
    OFFERED_FAILED = 100U,
    ALREADY_OFFERED = 101U,
    CONFIGURATION_MISMATCH = 102U,
    DEBOUNCING_CONFIGURATION_INCONSISTENT = 103U
};
class DiagOfferErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DiagOfferErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DiagOfferErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagOfferErrc::ALREADY_OFFERED):
                ret = "Already offered";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagOfferErrc::CONFIGURATION_MISMATCH):
                ret = "Configure mismatch";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagOfferErrc::DEBOUNCING_CONFIGURATION_INCONSISTENT):
                ret = "Debouncing configuration inconsistant";
                break;
            default:
                ret = "Unknown diag error";
                break;
        }

        return ret;
    }
private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DiagOfferErrorDomain g_diagOfferErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDiagOfferErrorDomain()
{
    return internal::g_diagOfferErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DiagOfferErrc& code,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(code), GetDiagOfferErrorDomain(),
        supportData);
}
}
}
#endif
