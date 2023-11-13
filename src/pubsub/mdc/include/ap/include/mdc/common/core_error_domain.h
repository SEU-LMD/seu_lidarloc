/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the implementation of ErrorCode class according to AutoSAR standard core type
 * Create: 2020-03-20
 */
#ifndef MDC_COMMON_CORE_ERROR_DOMAIN_H
#define MDC_COMMON_CORE_ERROR_DOMAIN_H
#include "mdc/common/error_domain.h"
namespace mdc {
namespace common {
enum class CoreErrc : ErrorDomain::CodeType {
    kInvalidArgument = 22,
    kInvalidMetaModelShortname = 137,
    kInvalidMetaModelPath = 138
};

class CoreErrorDomain final : public ErrorDomain {
public:
    using Errc = CoreErrc;
    constexpr CoreErrorDomain() noexcept
        : ErrorDomain(kId)
    {}
    ~CoreErrorDomain() = default;
    char const *Name() const noexcept override
    {
        return "Core";
    }
    char const *Message(const ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (static_cast<Errc>(errorCode)) {
            case Errc::kInvalidArgument:
                ret = "Invalid argument";
                break;
            case Errc::kInvalidMetaModelShortname:
                ret = "Invalid meta model short name";
                break;
            case Errc::kInvalidMetaModelPath:
                ret = "Invalid meta model path";
                break;
            default:
                ret = "Unknown core error";
                break;
        }

        return ret;
    }

private:
    constexpr static ErrorDomain::IdType kId = 0x8000000000000014;
};

namespace internal {
constexpr CoreErrorDomain g_coreErrorDomain;
}

constexpr ErrorDomain const& GetCoreDomain() noexcept
{
    return internal::g_coreErrorDomain;
}

constexpr ErrorCode MakeErrorCode(CoreErrc code, ErrorDomain::SupportDataType data) noexcept
{
    return ErrorCode(static_cast<ErrorDomain::CodeType>(code), GetCoreDomain(), data);
}
}
}

#endif
