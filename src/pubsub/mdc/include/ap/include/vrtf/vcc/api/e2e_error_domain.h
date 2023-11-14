/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2021-04-06
 */
#ifndef VRTF_VCC_API_E2E_ERROR_DOMAIN_H_
#define VRTF_VCC_API_E2E_ERROR_DOMAIN_H_
#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "vrtf/vcc/api/types.h"

namespace vrtf {
namespace vcc {
namespace api {
namespace types {
enum class E2EErrorCode: ara::core::ErrorDomain::CodeType {
    repeated = 1,
    wrong_sequence_error = 2,
    error = 3,
    not_available = 4,
    no_new_data = 5,
    check_disable = 6
};

class E2EException : public ara::core::Exception {
public:
    explicit E2EException(vrtf::core::ErrorCode && err) noexcept
        : Exception(std::move(err)) {}
    ~E2EException(void) = default;
};

class E2EErrorDomain final : public vrtf::core::ErrorDomain {
public:
    using Errc = E2EErrorCode;
    using Exception = E2EException;

    constexpr static ErrorDomain::IdType E2EErrorDomainId = 0x8000000000001268U; // SWS_CM_99026

    constexpr E2EErrorDomain() noexcept
        : ErrorDomain(E2EErrorDomainId) {}
    ~E2EErrorDomain(void) = default;
    char const *Name() const noexcept override
    {
        return "E2E";
    }
    char const *Message(vrtf::core::ErrorDomain::CodeType errorCode) const noexcept override
    {
        static std::map<vrtf::core::ErrorDomain::CodeType, const std::string> mapCode = {
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::repeated), "repeated"},
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::wrong_sequence_error), "wrong_sequence_error"},
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::error), "error"},
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::not_available), "not_available"},
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::no_new_data), "no_new_data"},
            {vrtf::core::ErrorDomain::CodeType(E2EErrorCode::check_disable), "check_disable"}
        };
        return mapCode[errorCode].c_str();
    }

    void ThrowAsException(vrtf::core::ErrorCode const &errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<Exception>(errorCode);
    }
};

constexpr E2EErrorDomain g_E2EErrorDomain;

constexpr vrtf::core::ErrorDomain const &GetE2EErrorDomain() noexcept
{
    return g_E2EErrorDomain;
}

constexpr vrtf::core::ErrorCode MakeErrorCode(E2EErrorCode code,
                                              vrtf::core::ErrorDomain::SupportDataType data) noexcept
{
    return vrtf::core::ErrorCode(static_cast<vrtf::core::ErrorDomain::CodeType>(code),
                                 GetE2EErrorDomain(), data);
}

using E2EDataId = ara::core::Vector<std::uint32_t>;
using MessageCounter = std::uint32_t;
}
}
}
}
#endif

