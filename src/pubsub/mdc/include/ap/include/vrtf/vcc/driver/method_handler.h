/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: MethodHandler in vcc
 * Create: 2019-11-19
 */
#ifndef VRTF_VCC_API_INTERNAL_METHODHANDLER_H
#define VRTF_VCC_API_INTERNAL_METHODHANDLER_H
#include <memory>
#include "vrtf/vcc/api/types.h"
namespace vrtf {
namespace vcc {
namespace driver {
class MethodHandler {
public:
    MethodHandler() {}
    virtual ~MethodHandler() {}
    /**
     * @brief Public base of Send request
     * @details Public base of Send request
     *
     * @param data A pointer to the first address of the sent data
     * @param length Length of data
     * @param sessionId Method counter
     * @return Whether the request was sent successfully
     *   @retval true Request sent successfully
     *   @retval false Request sent failed
     * @see SomeipMethodHandler::Request DDSMethodHandler::Request
     */
    virtual bool Request(uint8_t* data, const size_t &length,
        const vrtf::vcc::api::types::SessionId& sessionId) = 0;
    virtual bool Reply(uint8_t* data, const size_t &length,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, bool isUsingIncorrectId = false) = 0;
    virtual bool ReplyError(uint8_t* data, const size_t length,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, bool isUsingIncorrectId = false) = 0;
    virtual const uint8_t *AllocateBuffer(
        const uint32_t& length, const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg) = 0;
    virtual void ReturnLoan(const uint8_t *data) = 0;
    virtual void SetReceiveHandler(vrtf::vcc::api::types::MethodReceiveHandler handler) = 0;
    virtual vrtf::serialize::SerializeType GetSerializeType() = 0;
    virtual void SetMethodStateChangeHandler(const vrtf::vcc::api::types::MethodStateChangeHandler& handler) = 0;
    virtual void UnsetMethodStateChangeHandler() = 0;
};
}
}
}

#endif
