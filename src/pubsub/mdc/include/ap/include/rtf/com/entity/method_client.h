/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: MethodClient class
 * Create: 2020-04-22
 */
#ifndef RTF_COM_METHOD_CLIENT_H_
#define RTF_COM_METHOD_CLIENT_H_

#include <chrono>

#include "rtf/com/adapter/ros_proxy_adapter.h"
#include "rtf/com/types/method_result.h"
namespace rtf {
namespace com {
class MethodClient {
public:
    /**
     * @brief MethodClient default constructor
     */
    MethodClient(void) = default;

    /**
     * @brief MethodClient constructor
     * @param[in] adapter    The actual adapter that handles this method
     */
    MethodClient(const std::shared_ptr<adapter::RosProxyAdapter>& adapter);

    /**
     * @brief MethodClient destructor
     */
    ~MethodClient(void) = default;

    /**
     * @brief MethodClient copy constructor
     * @note deleted
     * @param[in] other    Other instance
     */
    MethodClient(const MethodClient& other) = delete;

    /**
     * @brief MethodClient move constructor
     * @param[in] other    Other instance
     */
    MethodClient(MethodClient && other);

    /**
     * @brief MethodClient copy assign operator
     * @note deleted
     * @param[in] other    Other instance
     */
    MethodClient& operator=(const MethodClient& other) = delete;

    /**
     * @brief MethodClient move assign operator
     * @param[in] other    Other instance
     */
    MethodClient& operator=(MethodClient && other);

    /**
     * @brief Returns whether of the method client is created correctly
     * @return Is the method client created correctly
     */
    operator bool() const noexcept;

    /**
     * @brief Returns whether of the method is avaliable
     * @return Is the method avaliable
     */
    bool IsValid(void) const noexcept;

    /**
     * @brief Make a remote procedure call
     * @param[in] methodData    whole method data structure
     * @param[in] timeout       rpc call timeout
     * @return rpc operation result
     */
    template <class MethodData, typename Rep, typename Period>
    bool Call(MethodData& methodData, const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        bool result = false;
        if (adapter_ != nullptr) {
            result = (adapter_->Call(methodData, timeout).GetErrorCode() == ErrorCode::OK);
        }
        return result;
    }

    /**
     * @brief Make a remote procedure call with E2E
     * @param[in] methodData    whole method data structure
     * @param[in] timeout       rpc call timeout
     * @return rpc operation result with E2E result
     */
    template <class MethodData, typename Rep, typename Period>
    MethodClientResult CallEx(MethodData& methodData, const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        MethodClientResult result;
        if (adapter_ != nullptr) {
            result = adapter_->Call(methodData, timeout);
        }
        return result;
    }

    /**
     * @brief close the connection
     * @return void
     */
    void Shutdown(void) noexcept;

private:
    std::shared_ptr<adapter::RosProxyAdapter> adapter_;
};
} // namespace com
} // namespace rtf
#endif // RTF_COM_METHOD_CLIENT_H_
