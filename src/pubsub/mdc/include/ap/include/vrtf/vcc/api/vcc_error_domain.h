/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2020-04-02
 */
#ifndef VCC_COM_ERROR_DOMAIN_H_
#define VCC_COM_ERROR_DOMAIN_H_
#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "vrtf/vcc/api/types.h"

namespace vrtf {
namespace vcc {
namespace api {
namespace types {
enum class VccErrc: vrtf::core::ErrorDomain::CodeType {
    OK = 0,
    ERROR = 1,
    INVALID_ARGUMENT = 2
};

class VccException : public ara::core::Exception {
public:
    explicit VccException(vrtf::core::ErrorCode && err) noexcept
        : Exception(std::move(err)) {}
    ~VccException(void) = default;
};

class VccErrorDomain final : public vrtf::core::ErrorDomain {
public:
    using Errc = VccErrc;
    using Exception = VccException;

    constexpr static  ErrorDomain::IdType VccErrorDomainId = 0x8000000000001268U;

    constexpr VccErrorDomain() noexcept
        : ErrorDomain(VccErrorDomainId) {}
    ~VccErrorDomain(void) = default;
    char const *Name() const noexcept override
    {
        return "Vcc";
    }
    char const *Message(vrtf::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static std::map<vrtf::core::ErrorDomain::CodeType, const std::string> mapCode = {
            {vrtf::core::ErrorDomain::CodeType(VccErrc::OK), "OK"},
            {vrtf::core::ErrorDomain::CodeType(VccErrc::ERROR), "ERROR"},
            {vrtf::core::ErrorDomain::CodeType(VccErrc::INVALID_ARGUMENT), "INVALID_ARGUMENT"}
        };
            return mapCode[errorCode].c_str();
    }

    void ThrowAsException(vrtf::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<Exception>(errorCode);
    }
};

constexpr VccErrorDomain g_VccErrorDomain;

constexpr vrtf::core::ErrorDomain const &GetVccErrorDomain() noexcept
{
    return g_VccErrorDomain;
}

constexpr vrtf::core::ErrorCode MakeErrorCode(VccErrc code,
                                              vrtf::core::ErrorDomain::SupportDataType data) noexcept
{
    return vrtf::core::ErrorCode(static_cast<vrtf::core::ErrorDomain::CodeType>(code),
                                 GetVccErrorDomain(), data);
}
}
}
}
}
#endif
