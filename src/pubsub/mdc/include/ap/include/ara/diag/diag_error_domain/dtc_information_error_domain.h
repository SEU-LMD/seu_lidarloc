/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag operation cycle error domain
 */
#ifndef ARA_DTC_INFORMATION_ERROR_DOMAIN_H
#define ARA_DTC_INFORMATION_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DtcInformationErrc : ara::core::ErrorDomain::CodeType {
    kGenericError = 107,
    kInvalidArgument = 105
};
class DtcInformationErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DtcInformationErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DtcInformationErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DtcInformationErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DtcInformationErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown dtc information error";
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
constexpr DtcInformationErrorDomain g_dtcInformationErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDtcInformationErrorDomain()
{
    return internal::g_dtcInformationErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DtcInformationErrc code,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDtcInformationErrorDomain(),
        supportData);
}
}
}
#endif
