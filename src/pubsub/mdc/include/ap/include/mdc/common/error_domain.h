/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of ErrorDomain class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef MDC_COMMON_ERROR_DOMAIN_H
#define MDC_COMMON_ERROR_DOMAIN_H
#include <cstdint>
namespace mdc {
namespace common {
class ErrorCode;

/**
 * @brief Encapsulation of an error domain [SWS_CORE_00110].
 *
 */
class ErrorDomain {
public:
    /**
     * @brief Alias type for a unique ErrorDomain identifier type [SWS_CORE_00121].
     *
     */
    using IdType = std::uint64_t;

    /**
     * @brief Alias type for a domain-specific error code value [SWS_CORE_00122].
     *
     */
    using CodeType = std::int32_t;

    /**
     * @brief Alias type for vendor-specific supplementary data [SWS_CORE_00123].
     *
     */
    using SupportDataType = std::int32_t;

    /**
     * @brief Copy construction shall be disabled [SWS_CORE_00131].
     *
     */
    ErrorDomain(ErrorDomain const &) = delete;

    /**
     * @brief Move construction shall be disabled [SWS_CORE_00132].
     *
     */
    ErrorDomain(ErrorDomain &&) = delete;

    /**
     * @brief Copy assignment shall be disabled [SWS_CORE_00133].
     *
     */
    ErrorDomain& operator=(ErrorDomain const &) = delete;

    /**
     * @brief Move assignment shall be disabled [SWS_CORE_00134].
     *
     */
    ErrorDomain& operator=(ErrorDomain &&) = delete;

    /**
     * @brief Compare for equality with another ErrorDomain instance [SWS_CORE_00137].
     *
     * @param[in]  other    the other instance
     * @return     bool     true if other is equal to *this, false otherwise
     */
    constexpr bool operator==(ErrorDomain const &other) const noexcept
    {
        return uniqueId_ == other.uniqueId_;
    }

    /**
     * @brief Compare for non-equality with another ErrorDomain instance [SWS_CORE_00138].
     *
     * @param[in]   other   the other instance
     * @return      bool    true if other is not equal to *this, false otherwise
     */
    constexpr bool operator!=(ErrorDomain const &other) const noexcept
    {
        return uniqueId_ != other.uniqueId_;
    }

    /**
     * @brief Return the unique domain identifier [SWS_CORE_00151].
     *
     * @return IdType the identifier
     */
    constexpr IdType Id() const noexcept
    {
        return uniqueId_;
    }

    virtual char const *Name() const noexcept = 0;
    virtual char const *Message(const CodeType& errorCode) const noexcept = 0;

protected:
    /**
     * @brief Construct a new instance with the given identifier [SWS_CORE_00135].
     *
     * @param[in] id the unique identifier
     */
    explicit constexpr ErrorDomain(const IdType id) noexcept : uniqueId_(id)
    {
    }

    /**
     * @brief Destructor [SWS_CORE_00136].
     *
     */
    ~ErrorDomain() = default;
private:
    IdType uniqueId_;
};
}
}
#endif
