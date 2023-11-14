/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: class Timer declaration.
 * The implementation of this class does not depend on the specific OS.
 */

#ifndef HCFI_TIMER_H
#define HCFI_TIMER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace mdc {
namespace hcfi {
// calc greatest common divisor
template <typename T> T Gcd(T a, T b, const T zero)
{
    while (b != zero) {
        T temp = a % b;
        a = b;
        b = temp;
    }
    return a;
}
class Timer {
public:
    // default 1ms
    static std::chrono::microseconds percision;

    // suggested mini sleepdelay for timer tick
    static std::chrono::microseconds tickSleepDelay;

    // The minimum delay accuracy of the sleep() function of the operating system
    static std::chrono::microseconds sleepMinPercision;

    Timer() : duration_(0U), lastTime_(), callback_(nullptr), isEnabled_(false), tickCount_(0U) {}
    ~Timer();
    Timer(Timer &&) = delete;
    Timer &operator = (Timer &&) = delete;

    // suggested the timeout period of the timer is an integer multiple of the minimum precision of the timer
    bool SetPeriod(const std::chrono::milliseconds dur);
    bool SetCallback(std::function<void()> const f);

    void Start();
    void Stop();
    size_t GetCount() const
    {
        return tickCount_;
    }

    struct TickThreadManager {
        TickThreadManager() : loopFlag(true) {}
        ~TickThreadManager()
        {
            loopFlag = false;
        }
        bool loopFlag;
    };

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

    static void TickThread();
    static void UpdateRequiredPercision();
    static void UpdateTimer();
    static void WaitForMinPercision(TimePoint &now, const TimePoint startTimePiont,
        std::chrono::microseconds &timeToWait);

    std::chrono::microseconds duration_;
    TimePoint lastTime_;
    std::function<void()> callback_;
    bool isEnabled_;
    size_t tickCount_;

    static std::vector<Timer *> timerList_;
    static std::mutex timerListMutex_;
    static std::thread tickThread_;
    static std::atomic<std::chrono::microseconds> requiredPercision_;
    static std::mutex tickThreadWakerMutex_;
    static std::condition_variable tickThreadWakerCV_;
    static TickThreadManager tickThreadMgr_;
};
} // namespace hcfi
} // namespace mdc

#endif
