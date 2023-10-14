/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2020-04-02
 */
#ifndef VRTF_VCC_API_DYNAMIC_ERROR_DOMAIN_H_
#define VRTF_VCC_API_DYNAMIC_ERROR_DOMAIN_H_
#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "vrtf/vcc/api/types.h"

namespace vrtf {
namespace vcc {
namespace api {
namespace types {
enum class DynamicErrc: vrtf::core::ErrorDomain::CodeType {
    default_errc = 0
};
DynamicErrc SwitchVccErrorToDynamicError(vrtf::vcc::api::types::ErrorCode code);

class DynamicException : public ara::core::Exception {
public:
    explicit DynamicException(vrtf::core::ErrorCode && err) noexcept
        : Exception(std::move(err)) {}
    ~DynamicException(void) = default;
};

class DynamicErrorDomain final : public vrtf::core::ErrorDomain {
public:
    using Errc = DynamicErrc;
    using Exception = DynamicException;
    // According to SWS_CORE_00016 Vendor-defined error domain range 0xc000’0000'0000'0000 ~ 0xc000’0000'ffff'ffff
    constexpr static  ErrorDomain::IdType DefaultDynamicErrorDomainId = 0xc000000000000001U;

    constexpr DynamicErrorDomain(const ErrorDomain::IdType& id = DefaultDynamicErrorDomainId) noexcept
        : ErrorDomain(id) {}
    ~DynamicErrorDomain(void) = default;
    char const *Name() const noexcept override
    {
        return "Dynamic";
    }
    char const *Message(vrtf::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static_cast<void>(errorCode);
        return "Dynamic error domain doesn't support message output";
    }

    void ThrowAsException(vrtf::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<Exception>(errorCode);
    }
};

constexpr DynamicErrorDomain g_DynamicErrorDomain;

constexpr vrtf::core::ErrorDomain const &GetDynamicErrorDomain() noexcept
{
    return g_DynamicErrorDomain;
}

constexpr vrtf::core::ErrorCode MakeErrorCode(DynamicErrc code,
                                              vrtf::core::ErrorDomain::SupportDataType data) noexcept
{
    return vrtf::core::ErrorCode(static_cast<vrtf::core::ErrorDomain::CodeType>(code),
                                 GetDynamicErrorDomain(), data);
}
}
}
}
}
#endif
