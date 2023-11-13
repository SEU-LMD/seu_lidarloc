/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of ThreadPoolHelper
 * Create: 2020-08-17
 */
#ifndef RTF_COM_THREAD_POOL_HELPER_H_
#define RTF_COM_THREAD_POOL_HELPER_H_
#include <mutex>
#include "rtf/com/entity/thread_group.h"
#include "rtf/com/types/ros_types.h"
namespace rtf {
namespace com {
namespace utils {
class ThreadPoolHelper {
public:
    /**
     * @brief Create a thread pool which index is inputted threadgroup
     *
     * @param[in] group           The index of new thread pool
     * @param[in] threadNumber    The thread number of the new thread pool
     * @param[in] queueSize       The queue size of the new thread pool
     */
    static bool AddThreadPool(const ThreadGroup& group) noexcept;

    /**
     * @brief Get the corresponding thread pool of the thread group
     *
     * @param[in] group   the index of thread pool
     * @return std::shared_ptr<VccThreadPool> query result
     */
    static std::shared_ptr<VccThreadPool> QueryThreadPool(const ThreadGroup& group) noexcept;

    /**
     * @brief Delete the Thread pool from helper(manager)
     *
     * @param[in] group  The index of the deleting thread pool
     */
    static void DeleteThreadPool(const ThreadGroup& group) noexcept;
private:
    static std::mutex threadPoolMapMutex_;
};
}
}
}
#endif
