/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: The declaration of VccMethodReturnType
 * Create: 2021-04-22
 */
#ifndef VRTF_VCC_API_VCC_METHOD_RETURN_TYPE_
#define VRTF_VCC_API_VCC_METHOD_RETURN_TYPE_

#include "ara/core/future.h"
#include "vrtf/vcc/api/param_struct_typse.h"

namespace vrtf {
namespace vcc {
namespace api {
template <class ReplyDataType>
class VccMethodReturnType {
public:
    /**
     * @brief Delete default constructor
     *
     */
    VccMethodReturnType() = delete;

    /**
     * @brief Copy construtor
     *
     */
    VccMethodReturnType(const VccMethodReturnType& other) = default;

    /**
     * @brief Copy assignment
     *
     */
    VccMethodReturnType& operator=(const VccMethodReturnType& replyFuture) & = default;

    /**
     * @brief Construct a new Vcc Method Return Type object
     *
     * @param[in] replyResult The reply result will be moved in the new object
     */
    explicit VccMethodReturnType(vrtf::core::Result<ReplyDataType> && replyResult)
        : VccMethodReturnType(replyResult, false) {}

    /**
     * @brief Construct a new Vcc Method Return Type object
     *
     * @param[in] error     The error code will set into reply result
     */
    explicit VccMethodReturnType(const vrtf::core::ErrorCode& error)
        : VccMethodReturnType(vrtf::core::Result<ReplyDataType>(error), false) {}

    /**
     * @brief Construct a new Vcc Method Return Type
     *
     * @param[in] replyResult           The Reply Result which contains reply data
     * @param[in] isUsingIncorrectId    Is Using Incorrect Id flag
     */
    VccMethodReturnType(vrtf::core::Result<ReplyDataType> && replyResult, const bool isUsingIncorrectId)
        : replyResult_(std::move(replyResult)), isUsingIncorrectId_(isUsingIncorrectId) {}

    /**
     * @brief Destroy the Vcc Method Return Type object
     *
     */
    ~VccMethodReturnType() = default;

    /**
     * @brief Get the Is Using Incorrect Id flag
     *
     * @return bool
     *      @retval true    Using incorrect E2E Id to protect the reponse
     *      @retval false   Do not use incorrect E2E Id to protect the response
     */
    bool GetIsUsingIncorrectId() const { return isUsingIncorrectId_; }

    /**
     * @brief Get the Reply Result which contains reply data
     *
     * @return const vrtf::core::Result<ReplyDataType>&   The Reply Result which contains reply data
     */
    const vrtf::core::Result<ReplyDataType>& GetReplyResult() const { return replyResult_; }
private:
    vrtf::core::Result<ReplyDataType> replyResult_;
    bool isUsingIncorrectId_ = false;
};

namespace internal {
template<typename T>
struct IsVccMethodReturnType: public std::false_type {};

template<typename T>
struct IsVccMethodReturnType<VccMethodReturnType<T>> : public std::true_type {};
}
}
} // namespace com
} // namespace rtf
#endif // RTF_COM_PUBLISHER_H_
