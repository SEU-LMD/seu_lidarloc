/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of FutureErrorDomain class according to AutoSAR standard core type
 * Create: 2019-07-30
 */
#ifndef MDC_COMMON_FUTURE_ERROR_DOMAIN_H
#define MDC_COMMON_FUTURE_ERROR_DOMAIN_H
#include <cstdint>

#include "mdc/common/error_domain.h"
#include "mdc/common/error_code.h"

namespace mdc {
namespace common {
/**
 * @brief Specifies the types of internal errors that can occur upon calling Future::get or Future::GetResult.
 * [SWS_CORE_00400]
 */
enum class future_errc : ErrorDomain::CodeType {
    broken_promise = 101,               // the asynchronous task abandoned its shared state
    future_already_retrieved = 102,     // the contents of the shared state were already accessed
    promise_already_satisfied = 103,    // attempt to store a value into the shared state twice
    no_state = 104,                     // attempt to access Promise or Future without an associated state
};

/**
 * @brief Error domain for errors originating from classes Future and Promise.
 *
 */
class FutureErrorDomain final : public ErrorDomain {
public:
    /**
     * @brief Alias for the error code value enumeration. [SWS_CORE_00431]
     *
     */
    using Errc = future_errc;

    /**
     * @brief Default constructor. [SWS_CORE_00441]
     *
     */
    constexpr FutureErrorDomain() noexcept
        : ErrorDomain(kId_)
    {}

    /**
     * @brief Destroy the Future Error Domain object
     *
     */
    ~FutureErrorDomain() = default;

    /**
     * @brief Return the "shortname" ApApplicationErrorDomain.SN of this error domain. [SWS_CORE_00442]
     *
     * @return char const*  "Future"
     */
    char const *Name() const noexcept override
    {
        return "Future";
    }

    /**
     * @brief Translate an error code value into a text message. [SWS_CORE_00443]
     *
     * @param errorCode[in]   the error code value
     * @return char const*    the text message, never nullptr
     */
    char const *Message(const ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const char* ret = nullptr;
        switch (static_cast<Errc>(errorCode)) {
            case Errc::broken_promise:
                ret = "broken promise";
                break;
            case Errc::future_already_retrieved:
                ret = "future already retrieved";
                break;
            case Errc::promise_already_satisfied:
                ret = "promise already satisfied";
                break;
            case Errc::no_state:
                ret = "no state";
                break;
            default:
                ret = "unknown future error";
                break;
        }

        return ret;
    }

private:
    constexpr static ErrorDomain::IdType kId_ = 0x8000000000000013;
};

namespace internal {
constexpr FutureErrorDomain g_futureErrorDomain;
}

/**
 * @brief Obtain the reference to the single global FutureErrorDomain instance [SWS_CORE_00480].
 *
 * @return ErrorDomain const&  reference to the FutureErrorDomain instance
 */
constexpr ErrorDomain const& GetFutureErrorDomain() noexcept
{
    return internal::g_futureErrorDomain;
}

/**
 * @brief Create a new ErrorCode for FutureErrorDomain with the given support data type.
 *
 * @param[in] code        an enumeration value from future_errc
 * @param[in] data        a vendor-defined supplementary value
 * @return    ErrorCode   the new ErrorCode instance
 */
constexpr ErrorCode MakeErrorCode(future_errc code, ErrorDomain::SupportDataType data) noexcept
{
    return ErrorCode(static_cast<ErrorDomain::CodeType>(code), GetFutureErrorDomain(), data);
}
}
}

#endif
