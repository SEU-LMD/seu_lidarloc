/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the implementation of ErrorCode class of diagnostic communication.
 */
#ifndef DIAG_COM_ERROR_DOMAIN_H
#define DIAG_COM_ERROR_DOMAIN_H
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
namespace mdc {
namespace diag {
enum class DiagComErrc : ara::core::ErrorDomain::CodeType {
    COMMUNICATION_ERROR = 0,
    INSTANCE_SPECIFIER_INVALID,
    SERIAL_NUMBER_INVALID,
    TIMEOUT
};

class DiagComErrorDomain final : public ara::core::ErrorDomain {
public:
    using Errc = DiagComErrc;
    constexpr DiagComErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagComErrorDomain() = default;
    char const *Name() const noexcept override
    {
        return "DiagCom";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char *retChar = nullptr;
        switch (static_cast<Errc>(errorCode)) {
            case Errc::COMMUNICATION_ERROR:
                retChar = "diag communication error!";
                break;
            case Errc::INSTANCE_SPECIFIER_INVALID:
                retChar = "instance specifier invalid";
                break;
            case Errc::SERIAL_NUMBER_INVALID:
                retChar = "serial number invalid";
                break;
            case Errc::TIMEOUT:
                retChar = "request timeout";
                break;
            default:
                retChar = "Unknown diag communication error";
                break;
        }
        return retChar;
    }
// not to throw exception
void ThrowAsException(ara::core::ErrorCode const &) const noexcept(false) override{}
private:
    constexpr static ara::core::ErrorDomain::IdType kId = 0x8000000000000001U; // need to design
};

namespace internal {
constexpr DiagComErrorDomain DIAG_COM_ERROR_DOMAIN;
}

constexpr ara::core::ErrorDomain const& GetDiagComErrorDomain() noexcept
{
    return internal::DIAG_COM_ERROR_DOMAIN;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagComErrc comErrorcode,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(comErrorcode), GetDiagComErrorDomain(),
        supportData);
}
}
}

#endif
