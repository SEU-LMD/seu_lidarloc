/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The Execution Error Code.
 * Create: 2020-04-10
 */

#ifndef VRTF_EXEC_ERROR_DOMAIN_H
#define VRTF_EXEC_ERROR_DOMAIN_H

#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
namespace ara  {
namespace exec {
enum class ExecErrc : ara::core::ErrorDomain::CodeType {
    kGeneralError = 1U,
    kInvalidArguments = 2U,
    kCommunicationError = 3U,
    kMetaModelError = 4U,
    kCancelled = 5U,
    kFailed = 6U,
    kBusy = 7U
};

class ExecException : public ara::core::Exception {
public:
    explicit ExecException(ara::core::ErrorCode && err) noexcept : Exception(std::move(err)) {}
    ~ExecException(void) = default;
};

class ExecErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr static ErrorDomain::IdType ExecErrcDomainId = 0x8000000000000300ULL;
    constexpr ExecErrorDomain() noexcept : ErrorDomain(ExecErrcDomainId) {}
    ~ExecErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Exec";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static std::map<ara::core::ErrorDomain::CodeType, std::string const> mapCode = {
            {ara::core::ErrorDomain::CodeType(ExecErrc::kGeneralError),       "Some unspecified error occurred"},
            {ara::core::ErrorDomain::CodeType(ExecErrc::kInvalidArguments),   "Invalid argument was passed"},
            {ara::core::ErrorDomain::CodeType(ExecErrc::kCommunicationError), "Communication error occurred"},
            {ara::core::ErrorDomain::CodeType(ExecErrc::kMetaModelError),
                "Wrong meta model identifier passed to a function"},
            {ara::core::ErrorDomain::CodeType(ExecErrc::kCancelled),
                "Transition to the requested Function Group state was cancelled by a newer request"},
            {ara::core::ErrorDomain::CodeType(ExecErrc::kFailed),
                "Transition to the requested Function Group state failed"}
        };
        return mapCode[errorCode].c_str();
    }
    void ThrowAsException(ara::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<ExecException>(errorCode);
    }
};

constexpr ExecErrorDomain g_ExecErrorDomain;
constexpr ara::core::ErrorDomain const &GetExecErrorDomain() noexcept
{
    return g_ExecErrorDomain;
}
constexpr ara::core::ErrorCode MakeErrorCode (ara::exec::ExecErrc code,
                                              ara::core::ErrorDomain::SupportDataType data = 0) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetExecErrorDomain(), data);
}
} // namespace exec
} // namespace ara

#endif // VRTF_EXEC_ERROR_DOMAIN_H

