/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Method result definition
 * Create: 2020-12-24
 */
#ifndef RTF_COM_TYPES_METHOD_RESULT_H_
#define RTF_COM_TYPES_METHOD_RESULT_H_

#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
#include "rtf/com/config/e2e/e2e_config.h"
#include "rtf/com/types/error_code.h"
#include "rtf/com/types/error_code_ex.h"

namespace rtf {
namespace com {
class MethodResult {
public:
    MethodResult(void) = default;

    ~MethodResult(void) = default;

    rtf::com::e2e::Result GetE2EResult() const noexcept { return e2eResult_; }

    void SetE2EResult(const rtf::com::e2e::Result& result) noexcept { e2eResult_ = result; }
private:
    rtf::com::e2e::Result e2eResult_ = rtf::com::e2e::Result(rtf::com::e2e::ProfileCheckStatus::kNotAvailable,
                                                             rtf::com::e2e::SMState::kStateMDisabled);
};

class MethodClientResult : public MethodResult {
public:
    MethodClientResult(void) = default;

    ~MethodClientResult(void) = default;

    ErrorCode GetErrorCode() const noexcept { return errorCode_; }

    void SetErrorCode(const ErrorCode& result) noexcept { errorCode_ = result; }

    ErrorCodeEx GetErrorCodeEx() const noexcept { return errorCodeEx_; }

    void SetErrorCodeEx(const ErrorCodeEx& result) noexcept { errorCodeEx_ = result; }

    std::string GetErrorMessage() const noexcept { return errorMessage_; }

    void SetErrorMessage(std::string const &errorMessage) noexcept { errorMessage_ = errorMessage; }
private:
    ErrorCode errorCode_ = ErrorCode::NOTAVAILABLE;
    ErrorCodeEx errorCodeEx_ = ErrorCodeEx(0, 0);
    // dfx help info
    std::string errorMessage_;
};


class MethodServerResult : public MethodResult {
public:
    MethodServerResult(void) = default;

    ~MethodServerResult(void) = default;

    /**
     * @brief Set the Using Incorrect E2E Id object, which is setted in config
     *
     * @param[in] isUsing    Is using incorrect E2E Id to protect this response
     */
    void SetUsingIncorrectE2EId(const bool isUsing) noexcept { isUsingIncorrectE2EId_ = isUsing; }

    /**
     * @brief Get the result that is using incorrect E2E Id to protect this response, which is setted in config
     *
     * @return bool
     *      @retval true    Using incorrect E2E Id to protect the reponse
     *      @retval false   Do not use incorrect E2E Id to protect the response
     */
    bool IsUsingIncorrectE2EId() const noexcept { return isUsingIncorrectE2EId_; }
private:
    bool isUsingIncorrectE2EId_ = false;
};
} // namespace com
} // namespace rtf
#endif
