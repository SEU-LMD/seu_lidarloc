/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of FutureErrorDomain class according to AutoSAR standard core type
 * Create: 2019-07-30
 */
#ifndef ARA_CORE_FUTURE_ERROR_DOMAIN_H
#define ARA_CORE_FUTURE_ERROR_DOMAIN_H

#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "ara/core/abort.h"
#include <cstdint>
#ifdef AOS_TAINT
#ifndef COVERITY_TAINT_SET_DEFINITION
#define COVERITY_TAINT_SET_DEFINITION
/**
 * @brief Function for Stain Modeling
 * @details The function is used only when the compilation macro AOS_TAINT is enabled.
 */
static void Coverity_Tainted_Set(void *buf){}
#endif
#endif
namespace ara {
namespace core {
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
 * @brief Exception type thrown by Future and Promise classes [SWS_CORE_00411].
 *
 */
class FutureException : public Exception {
public:
    explicit FutureException(ErrorCode err) noexcept
        : Exception(std::move(err))
    {
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&err);
#endif
    }
    ~FutureException() = default;
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
     * @brief Alias for the exception base class. [SWS_CORE_00432]
     *
     */
    using Exception = FutureException;

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
    char const *Message(ErrorDomain::CodeType errorCode) const noexcept override
    {
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&errorCode);
#endif
        switch (static_cast<Errc>(errorCode)) {
            case Errc::broken_promise:
                return "broken promise";
            case Errc::future_already_retrieved:
                return "future already retrieved";
            case Errc::promise_already_satisfied:
                return "promise already satisfied";
            case Errc::no_state:
                return "no state";
            default:
                ara::core::Abort("unknown future error");
                return "unknown future error";
        }
    }

    /**
     * @brief Throw the exception type corresponding to the given ErrorCode. [SWS_CORE_00444]
     *
     * @param errorCode the ErrorCode instance
     */
    void ThrowAsException(ErrorCode const & errorCode) const noexcept(false) override
    {
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&errorCode);
#endif
        ThrowOrTerminate<Exception>(errorCode);
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
constexpr ErrorDomain const & GetFutureErrorDomain() noexcept
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
