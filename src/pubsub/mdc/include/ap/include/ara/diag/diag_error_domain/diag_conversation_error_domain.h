/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag Conversation error domain
 */
#ifndef ARA_DIAG_CONVERSATION_ERROR_DOMAIN_H
#define ARA_DIAG_CONVERSATION_ERROR_DOMAIN_H
#include <cstdint>
#include "ara/core/error_code.h"
#include "ara/core/error_domain.h"
namespace ara {
namespace diag {
enum class DiagConversationErrc : ara::core::ErrorDomain::CodeType {
    kInvalidArgument = 105,
    kGenericError = 107
};
class DiagConversationErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr DiagConversationErrorDomain() noexcept
        : ara::core::ErrorDomain(kId)
    {}
    ~DiagConversationErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagConversationErrc::kInvalidArgument):
                ret = "Invalid Argument";
                break;
            case static_cast<ara::core::ErrorDomain::CodeType>(DiagConversationErrc::kGenericError):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag Conversation error";
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
constexpr DiagConversationErrorDomain g_diagConversationErrorDomain;
}

constexpr ara::core::ErrorDomain const& GetDiagConversationErrorDomain()
{
    return internal::g_diagConversationErrorDomain;
}

constexpr ara::core::ErrorCode MakeErrorCode(const DiagConversationErrc code,
    const ara::core::ErrorDomain::SupportDataType supportData = ara::core::ErrorDomain::SupportDataType()) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetDiagConversationErrorDomain(),
        supportData);
}
}
}
#endif
