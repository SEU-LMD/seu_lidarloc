/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Exception class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef ARA_CORE_EXPETION_H
#define ARA_CORE_EXPETION_H
#include <exception>
#include "ara/core/error_code.h"
namespace ara {
namespace core {
/**
 * @brief Base type for all AUTOSAR exception types [SWS_CORE_00601].
 *
 */
class Exception : public std::exception {
public:
    /**
     * @brief Construct a new Exception object [SWS_CORE_00611]
     *
     * @param[in] err   the ErrorCode
     */
    explicit Exception(ErrorCode err) noexcept : theErrorCode (err)
    {
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&err);
#endif
    }

    /**
     * @brief Destroy the Exception object
     *
     */
    virtual ~Exception() {}

    /**
     * @brief Return the explanatory string [SWS_CORE_00612].
     *
     * @return char const*   a null-terminated string
     */
    char const *what() const noexcept override
    {
        return theErrorCode.Message().data();
    }

    /**
     * @brief Return the embedded ErrorCode that was given to the constructor [SWS_CORE_00613].
     *
     * @return ErrorCode const&   reference to the embedded ErrorCode
     */
    ErrorCode const & Error() const noexcept
    {
        return theErrorCode;
    }
private:
    ErrorCode const theErrorCode;
};
}
}

#endif
