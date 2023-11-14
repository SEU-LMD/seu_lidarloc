/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag offer error domain
 */
#ifndef ARA_DIAG_OFFER_ERROR_DOMAIN_H
#define ARA_DIAG_OFFER_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DiagOfferErrc : ara::core::ErrorDomain::CodeType {
    kOfferedFailed = 100U,
    kAlreadyOffered = 101U,
    kConfigurationMismatch = 102U,
    kDebouncingConfigurationInconsistent = 103U
};
class DiagOfferErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DiagOfferErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagOfferErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagOfferErrc::kAlreadyOffered):
                ret = "Already offered";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagOfferErrc::kConfigurationMismatch):
                ret = "Configure mismatch";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagOfferErrc::kDebouncingConfigurationInconsistent):
                ret = "Debouncing configuration inconsistant";
                break;
            default:
                ret = "Unknown diag error";
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
constexpr DiagOfferErrorDomain g_diagOfferErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDiagOfferErrorDomain()
{
    return internal::g_diagOfferErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagOfferErrc code,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDiagOfferErrorDomain(),
        supportData);
}
}
}
#endif
