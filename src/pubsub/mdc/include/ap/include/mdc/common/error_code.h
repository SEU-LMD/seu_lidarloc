/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of ErrorCode class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef MDC_COMMON_ERROR_CODE_H
#define MDC_COMMON_ERROR_CODE_H
#include <iostream>
#include "mdc/common/error_domain.h"

namespace mdc {
namespace common {
/**
 * @brief Encapsulation of an error code [SWS_CORE_00501].
 *
 */
class ErrorCode final {
public:
    ErrorCode() = default;
    /**
     * @brief Construct a new ErrorCode instance with parameters [SWS_CORE_00512].
     *
     * @tparam      EnumT    an enum type that contains error code values
     * @param[in]   e        a domain-specific error code value
     * @param[in]   data     optional vendor-specific supplementary error context data
     */
    template <typename EnumT, typename = typename std::enable_if<std::is_enum<EnumT>::value>::type>
    constexpr ErrorCode(EnumT e, ErrorDomain::SupportDataType data = ErrorDomain::SupportDataType()) noexcept
        : ErrorCode(MakeErrorCode(e, data)) {}

    /**
     * @brief Construct a new ErrorCode instance with parameters [SWS_CORE_00513].
     *
     * @param[in]   value     a domain-specific error code value
     * @param[in]   domain    the ErrorDomain associated with value
     * @param[in]   data      optional vendor-specific supplementary error context data
     */
    constexpr ErrorCode(const ErrorDomain::CodeType& value,
                        ErrorDomain const& domain,
                        const ErrorDomain::SupportDataType& data = ErrorDomain::SupportDataType()) noexcept
        : errorCodeValue_(value), myDomain_(&domain), myData_(data) {}

    /**
     * @brief Destroy the Error Code object
     *
     */
    ~ErrorCode() = default;

    /**
     * @brief Return the raw error code value [SWS_CORE_00514].
     *
     * @return ErrorDomain::CodeType   the raw error code value
     */
    constexpr ErrorDomain::CodeType Value() const noexcept
    {
        return errorCodeValue_;
    }

    /**
     * @brief Return the domain with which this ErrorCode is associated [SWS_CORE_00515].
     *
     * @return ErrorDomain const&   the ErrorDomain
     */
    constexpr ErrorDomain const& Domain() const noexcept
    {
        return *myDomain_;
    }

    /**
     * @brief Return the supplementary error context data [SWS_CORE_00516].
     *
     * @return ErrorDomain::SupportDataType   the supplementary error context data
     */
    constexpr ErrorDomain::SupportDataType SupportData() const noexcept
    {
        return myData_;
    }

    /**
     * @brief Return a textual representation of this ErrorCode [SWS_CORE_00518].
     *
     * @return StringView   the error message text
     */
    std::string Message() const noexcept
    {
        const auto ret = Domain().Message(Value());
        if (ret == nullptr) {
            return "";
        }
        return ret;
    }

    /**
     * @brief Throw this error as exception [SWS_CORE_00519].
     *
     */
    void ThrowAsException() const;

private:
    ErrorDomain::CodeType errorCodeValue_;
    ErrorDomain const *myDomain_;
    ErrorDomain::SupportDataType myData_;

    friend std::ostream& operator<<(std::ostream& errorDomainOut, ErrorCode const& err)
    {
        return errorDomainOut << err.myDomain_->Name() << ":" << err.errorCodeValue_ << ":" << err.myData_;
    }
};

/**
 * @brief Global operator== for ErrorCode [SWS_CORE_00571].
 *
 * @param[in] lhs    the left hand side of the comparison
 * @param[in] rhs    the right hand side of the comparison
 * @return    bool   true if the two instances compare equal, false otherwise
 */
constexpr inline bool operator ==(ErrorCode const& lhs, ErrorCode const& rhs) noexcept
{
    return (lhs.Domain() == rhs.Domain()) && (lhs.Value() == rhs.Value());
}

/**
 * @brief Global operator!= for ErrorCode [SWS_CORE_00572].
 *
 * @param[in]  lhs     the left hand side of the comparison
 * @param[in]  rhs     the right hand side of the comparison
 * @return     bool    true if the two instances compare not equal, false otherwise
 */
constexpr inline bool operator !=(ErrorCode const& lhs, ErrorCode const& rhs)
{
    return (lhs.Domain() != rhs.Domain()) || (lhs.Value() != rhs.Value());
}
}
}
#endif
