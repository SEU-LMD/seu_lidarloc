/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The Phm Error Code.
 * Create: 2020-11-04
 */

#ifndef VRTF_PHM_ERROR_DOMAIN_H
#define VRTF_PHM_ERROR_DOMAIN_H

#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
namespace ara  {
namespace phm {
enum class PhmErrc : ara::core::ErrorDomain::CodeType {
    kGeneralError = 1U,
    kInvalidArguments = 2U,
    kCommunicationError = 3U,
};

class PhmException : public ara::core::Exception {
public:
    explicit PhmException(ara::core::ErrorCode && err) noexcept : Exception(std::move(err)) {}
    ~PhmException(void) = default;
};

class PhmErrorDomain final : public ara::core::ErrorDomain {
public:
    constexpr static ErrorDomain::IdType PhmErrcDomainId = 0x8000000000000399ULL; // temp phm id
    constexpr PhmErrorDomain() noexcept : ErrorDomain(PhmErrcDomainId) {}
    ~PhmErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "Phm";
    }
    char const *Message(ara::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static std::map<ara::core::ErrorDomain::CodeType, std::string const> mapCode = {
            {ara::core::ErrorDomain::CodeType(PhmErrc::kGeneralError),       "Some unspecified error occurred"},
            {ara::core::ErrorDomain::CodeType(PhmErrc::kInvalidArguments),   "Invalid argument was passed"},
            {ara::core::ErrorDomain::CodeType(PhmErrc::kCommunicationError), "Communication error occurred"}
        };
        return mapCode[errorCode].c_str();
    }
    void ThrowAsException(ara::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<PhmException>(errorCode);
    }
};

constexpr PhmErrorDomain g_PhmErrorDomain;
constexpr ara::core::ErrorDomain const &GetPhmErrorDomain() noexcept
{
    return g_PhmErrorDomain;
}
constexpr ara::core::ErrorCode MakeErrorCode (ara::phm::PhmErrc code,
                                              ara::core::ErrorDomain::SupportDataType data = 0) noexcept
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetPhmErrorDomain(), data);
}
} // namespace phm
} // namespace ara

#endif // VRTF_PHM_ERROR_DOMAIN_H

