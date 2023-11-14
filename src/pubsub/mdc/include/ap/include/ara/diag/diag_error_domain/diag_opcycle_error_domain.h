/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag operation cycle error domain
 */
#ifndef ARA_DIAG_OPCYCLE_ERROR_DOMAIN_H
#define ARA_DIAG_OPCYCLE_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DiagOpcycleErrc : ara::core::ErrorDomain::CodeType {
    kGenericError = 107,
    kInvalidArgument = 105
};
class DiagOpcycleErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DiagOpcycleErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagOpcycleErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagOpcycleErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagOpcycleErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag opcycle error";
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
constexpr DiagOpcycleErrorDomain g_diagOpcycleErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDiagOpcycleErrorDomain()
{
    return internal::g_diagOpcycleErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagOpcycleErrc code,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDiagOpcycleErrorDomain(),
        supportData);
}
}
}
#endif
