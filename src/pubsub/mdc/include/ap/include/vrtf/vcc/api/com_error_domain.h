/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2020-04-02
 */
#ifndef VRTF_VCC_API_COM_ERROR_DOMAIN_H_
#define VRTF_VCC_API_COM_ERROR_DOMAIN_H_
#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "vrtf/vcc/api/types.h"

namespace vrtf {
namespace vcc {
namespace api {
namespace types {
enum ComErrc: vrtf::core::ErrorDomain::CodeType {
    ok = 0,
    kServiceNotAvailable = 1,
    kMaxSamplesExceeded = 2,
    kNetworkBindingFailure = 3,
    kWrongMethodCallProcessingMode = 17  // Skeleton cannot be kEvent if using E2E in Method
};

class ComException : public ara::core::Exception {
public:
    explicit ComException(vrtf::core::ErrorCode && err) noexcept
        : Exception(std::move(err)) {}
    ~ComException(void) = default;
};

class ComErrorDomain final : public vrtf::core::ErrorDomain {
public:
    using Errc = ComErrc;
    using Exception = ComException;

    constexpr static  ErrorDomain::IdType ComErrorDomainId = 0x8000000000001267U; // SWS_CM_11267

    constexpr ComErrorDomain() noexcept
        : ErrorDomain(ComErrorDomainId) {}
    ~ComErrorDomain(void) = default;
    char const *Name() const noexcept override
    {
        return "Com";
    }
    char const *Message(vrtf::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static std::map<vrtf::core::ErrorDomain::CodeType, const std::string> mapCode = {
            {vrtf::core::ErrorDomain::CodeType(ComErrc::ok), "ok"},
            {vrtf::core::ErrorDomain::CodeType(ComErrc::kServiceNotAvailable), "kServiceNotAvailable"},
            {vrtf::core::ErrorDomain::CodeType(ComErrc::kMaxSamplesExceeded), "kMaxSamplesExceeded"},
            {vrtf::core::ErrorDomain::CodeType(ComErrc::kNetworkBindingFailure), "kNetworkBindingFailure"},
            {vrtf::core::ErrorDomain::CodeType(ComErrc::kWrongMethodCallProcessingMode),
                "kWrongMethodCallProcessingMode"}
        };
            return mapCode[errorCode].c_str();
    }

    void ThrowAsException(vrtf::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<Exception>(errorCode);
    }
};

constexpr ComErrorDomain g_ComErrorDomain;

constexpr vrtf::core::ErrorDomain const &GetComErrorDomain() noexcept
{
    return g_ComErrorDomain;
}

constexpr vrtf::core::ErrorCode MakeErrorCode(ComErrc code,
                                              vrtf::core::ErrorDomain::SupportDataType data) noexcept
{
    return vrtf::core::ErrorCode(static_cast<vrtf::core::ErrorDomain::CodeType>(code),
                                 GetComErrorDomain(), data);
}
}
}
}
}
#endif

