/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: diag conversation error domain
 */
#ifndef MDC_DIAG_CONVERSATION_ERROR_DOMAIN_H
#define MDC_DIAG_CONVERSATION_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DiagConversationErrc : mdc::common::ErrorDomain::CodeType {
    INVALID_ARGUMENT = 105,
    GENERIC_ERROR = 107
};
class DiagConversationErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DiagConversationErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DiagConversationErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Diag";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (errorCode) {
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagConversationErrc::INVALID_ARGUMENT):
                ret = "Invalid Argument";
                break;
            case static_cast<mdc::common::ErrorDomain::CodeType>(DiagConversationErrc::GENERIC_ERROR):
                ret = "Generic Error";
                break;
            default:
                ret = "Unknown diag conversation error";
                break;
        }

        return ret;
    }
private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FFU; // need to design
};

namespace internal {
constexpr DiagConversationErrorDomain g_diagConversationErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDiagConversationErrorDomain()
{
    return internal::g_diagConversationErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DiagConversationErrc& code,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(code), DiagConversationErrorDomain(),
        supportData);
}
}
}
#endif
