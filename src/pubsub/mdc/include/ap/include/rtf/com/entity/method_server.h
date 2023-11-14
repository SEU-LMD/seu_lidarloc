/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: MethodServer class
 * Create: 2020-04-22
 */
#ifndef RTF_COM_METHOD_SERVER_H_
#define RTF_COM_METHOD_SERVER_H_

#include "rtf/com/adapter/ros_skeleton_adapter.h"

namespace rtf {
namespace com {
class MethodServer {
public:
    /**
     * @brief MethodClient default constructor
     */
    MethodServer(void) = default;

    /**
     * @brief MethodServer constructor
     * @param[in] adapter    The actual adapter that handles this method
     */
    MethodServer(const std::shared_ptr<adapter::RosSkeletonAdapter>& adapter);

    /**
     * @brief MethodServer copy constructor
     * @note deleted
     * @param[in] other    Other instance
     */
    MethodServer(const MethodServer& other) = delete;

    /**
     * @brief MethodServer move constructor
     * @param[in] other    Other instance
     */
    MethodServer(MethodServer && other);

    /**
     * @brief MethodServer copy assign operator
     * @note deleted
     * @param[in] other    Other instance
     */
    MethodServer& operator=(const MethodServer& other) = delete;

    /**
     * @brief MethodServer move assign operator
     * @param[in] other    Other instance
     */
    MethodServer& operator=(MethodServer && other);

    /**
     * @brief MethodServer destructor
     */
    ~MethodServer(void) = default;

    /**
     * @brief Returns whether of the method server is created correctly
     * @return Is the method server created correctly
     */
    operator bool() const noexcept;

    /**
     * @brief close the connection
     * @return void
     */
    void Shutdown(void) noexcept;

private:
    std::shared_ptr<adapter::RosSkeletonAdapter> adapter_;
};
} // namespace com
} // namespace rtf
#endif // RTF_COM_METHOD_SERVER_H_
