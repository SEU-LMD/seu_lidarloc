/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of ThreadGroup
 * Create: 2020-08-17
 */
#ifndef RTF_COM_THREAD_GROUP_H_
#define RTF_COM_THREAD_GROUP_H_

#include <cstdint>
#include <atomic>
#include <memory>
#include <string>

#include "rtf/com/types/ros_types.h"

namespace rtf {
namespace com {
namespace utils {
class Logger;
}
class ThreadGroup {
public:
    /**
     * @brief Construct a new Thread Group object
     *
     * @param[in] threadNumber  The number of thread in the thread group
     * @param[in] queueSize     The queue size of the thread group
     * @param[in] mode          The thread mode of the thread group
     */
    ThreadGroup(std::uint16_t threadNumber = 1, std::uint16_t queueSize = 1024, ThreadMode mode = ThreadMode::EVENT);

    /**
     * @brief Destroy the Thread Group object
     *
     */
    ~ThreadGroup();

    /**
     * @brief Initailize the thread group
     *
     * @return Initialization result
     */
    bool Init() noexcept;

    /**
     * @brief Get the Thread Number object
     *
     * @return std::uint16_t  The thread number
     */
    std::uint16_t GetThreadNumber() const noexcept;

    /**
     * @brief Get the Queue Size object
     *
     * @return std::uint16_t  The size of queue
     */
    std::uint16_t GetQueueSize() const noexcept;

    /**
     * @brief operator< of ThreadGroup
     *
     * @param other    The other compared object
     * @return comparation result
     */
    bool operator<(const ThreadGroup& other) const noexcept;

    /**
     * @brief The ThreadGroup if is valid
     *
     * @return if the thread group is valid
     */
    bool IsValid(void) const noexcept;

    /**
     * @brief Free thread resources
     *
     */
    void Release(void) noexcept;

    /**
     * @brief Insert callback to get event data using SpinOnce. External user should never use.
     *
     */
    void AddSpinCallback(std::function<void()> const &cb) noexcept;
    /**
     * @brief Get callbacks to get event data using SpinOnce. External user should never use.
     *
     */
    std::vector<std::function<void()>> GetSpinCallbacks() const noexcept;
    /**
     * @brief Get thread mode. External user should never use.
     *
     */
    ThreadMode GetThreadMode() const noexcept;
private:
    std::uint16_t threadNumber_;
    std::uint16_t queueSize_;
    std::uint32_t groupId_;
    bool isValid_;
    static std::atomic<std::uint32_t> uid_;
    ThreadMode threadMode_;
    std::vector<std::function<void()>> spinCallbacks_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    static std::mutex spinCallbackMutex_;
};
}
}
#endif
