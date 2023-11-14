/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Htimer definition
 * Create: 2020-4-7
 */
#ifndef ARA_RTF_COMMON_HTIMER_H
#define ARA_RTF_COMMON_HTIMER_H

#include <stdio.h>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <condition_variable>

namespace ara {
namespace rtf {
namespace common {
class RtfTimer {
public:
    RtfTimer(std::string const &timerName = "");
    ~RtfTimer();

    bool Start(uint32_t msTime, std::function<void()> const &task, bool loop = false, bool async = true);

    void Cancel();

    template<typename callable, typename... arguments>
    bool SyncOnce(uint32_t msTime, callable && fun, arguments &&... args)
    {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(
            std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        task_ = std::move(task);
        return Start(msTime, task_, false, false);
    }

    template<typename callable, typename... arguments>
    bool AsyncOnce(uint32_t msTime, callable && fun, arguments &&... args)
    {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(
            std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        task_ = std::move(task);
        return Start(msTime, task_, false);
    }

    template<typename callable, typename... arguments>
    bool AsyncLoop(uint32_t msTime, callable && fun, arguments &&... args)
    {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(
            std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        task_ = std::move(task);
        return Start(msTime, task_, true);
    }

    void Reset(uint32_t msTime);
private:
    void DeleteThread();
    void Loop(std::function<void()> const &task);

    std::string name_;
    std::atomic_bool expired_;
    std::atomic_bool tryExpired_;
    std::atomic_bool loop_;
    std::atomic_bool resetFlag_;

    std::shared_ptr<std::thread> thread_;
    std::mutex threadLock_;
    std::condition_variable_any threadCon_;
    std::function<void()> task_;
    uint32_t msTime_;
};
} // namespace common
} // namespace rtf
} // namespace ara
#endif
