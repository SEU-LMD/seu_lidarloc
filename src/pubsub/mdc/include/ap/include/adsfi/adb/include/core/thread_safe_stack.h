/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  thread safe stack
 */
#ifndef THREAD_SAFE_STACK_H
#define THREAD_SAFE_STACK_H
#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>
#include <chrono>
namespace Adsfi {
/**
 * @brief 数据容器stack，后入先出，线程安全。保持容量在指定范围。保持数据在存活时间内。
 *        内部使用deque来实现。新的数据在back方向，旧的数据在front方向。
 *        功能描述如下：
 *        1. 容器数据定长。容器中只保存指定长度的数据，超过长度的数据会被删除。通过成员变量 capacityLimit 指定。
 *        2. 数据存活时间。容器中只保存存活时间以内的数据，超过存活时间的数据会被删除。通过成员变量 timeToLive 来指定。
 *           timeToLive参数为0或负值时，存活时间功能不开启，数据不会主动删除；为正值时，删除功能开启。
 *
 * @tparam T 数据类型。推荐使用智能指针
 */
template <typename T>
class ThreadSafeStack {
public:
    explicit ThreadSafeStack(const int64_t timeToLive0 = 0, const size_t capLimit = 5U)
        : timeToLive(timeToLive0), capacityLimit(capLimit)
    {}
    ~ThreadSafeStack() = default;
    ThreadSafeStack(const ThreadSafeStack&) = delete;
    ThreadSafeStack& operator=(const ThreadSafeStack&) = delete;
    void Push(const T& newValue)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        ClearOutnumberedData();

        container.push_back(newValue);
        times.push_back(std::chrono::steady_clock::now());
    }
    void Push(T&& newValue)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        ClearOutnumberedData();

        container.push_back(std::forward<T>(newValue));
        times.push_back(std::chrono::steady_clock::now());
    }
    /**
     * @brief 从容器中取出1帧数据，并将其从历史数据从去掉
     *
     * @return std::shared_ptr<T> 对数据加封装一层指针
     */
    std::shared_ptr<T> Pop()
    {
        std::unique_lock<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        std::shared_ptr<T> res(std::make_shared<T>(container.back()));
        PopBackData();
        return res;
    }
    /**
     * @brief 从缓存区获取N帧历史数据。注意：当停止向缓冲区写入数据后，历史数据不会自动清除。
     *
     * @param n pices of data
     * @return std::vector<std::shared_ptr<T>> newer data in the front
     */
    std::vector<std::shared_ptr<T>> GetNdata(const size_t n)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        auto len = std::min(capacityLimit, n);
        len = std::min(len, container.size());
        std::vector<std::shared_ptr<T>> out;
        out.reserve(len);
        auto bound = container.rbegin();
        std::advance(bound, len);
        for (auto it = container.rbegin(); (it != container.rend()) && (it != bound); ++it) {
            out.emplace_back(std::make_shared<T>(std::ref(*it)));
        }
        return out;
    }
    /**
     * @brief 从缓存区获取1帧历史数据。注意：当停止向缓冲区写入数据后，历史数据不会自动清除。
     *        container为空时返回nullptr
     *        补充功能描述：
     *        1. “新鲜”数据功能。有的情况下，非阻塞式接口也要获取“新鲜”的数据。要求这些数据的存入时间不能超过一定阈值。
     *              为满足这个需求，请设置入参 freshDataTime 获取指定时间内的数据（单位：ms)
     *              当不指定这个参数时，所有历史数据均可获取。
     *
     *
     * @return std::shared_ptr<T> return nullptr if container is empty.
     */
    std::shared_ptr<T> GetOneData(const uint32_t freshDataTime = UINT32_MAX)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        const auto oldtime = times.back();
        const auto now = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldtime).count();
        if (duration > static_cast<decltype(duration)>(freshDataTime)) {
            // no fresh data now
            return nullptr;
        }
        std::shared_ptr<T> data(std::make_shared<T>(container.back()));
        return data;
    }
    // 返回最旧的一帧数据，erase为true则删除最旧一帧数据
    std::shared_ptr<T> GetLastOne(const bool erase = false)
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        if (container.empty()) {
            return nullptr;
        }
        std::shared_ptr<T> data(std::make_shared<T>(container.front()));
        if (erase) {
            PopFrontData();
        }
        return data;
    }
    bool Empty()
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        return container.empty();
    }
    size_t Size()
    {
        std::lock_guard<std::mutex> lk(mut);
        ClearOldData();
        return container.size();
    }
    void Clear()
    {
        std::lock_guard<std::mutex> lk(mut);
        container.clear();
        times.clear();
    }

private:
    // 这个函数在调用的时候，清理所有超时的数据
    // 由于涉及数据修改，该函数必须在上锁之后调用
    void ClearOldData()
    {
        if (timeToLive <= 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        // 从最早放入的数据开始清理
        while (!times.empty()) {
            const auto oldtime = times.front();
            const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldtime).count();
            if (duration > timeToLive) {
                PopFrontData();
            } else {
                break;
            }
        }
    }
    // 清理所有超过容量限制的数据
    // 由于涉及数据修改，该函数必须在上锁之后调用
    inline void ClearOutnumberedData()
    {
        while (container.size() > capacityLimit) {
            PopFrontData();
        }
    }
    // 删除最新的一帧数据
    // 由于涉及数据修改，该函数必须在上锁之后调用
    inline void PopBackData()
    {
        container.pop_back();
        times.pop_back();
    }
    // 删除最老的一帧数据
    // 由于涉及数据修改，该函数必须在上锁之后调用
    inline void PopFrontData()
    {
        container.pop_front();
        times.pop_front();
    }

private:
    mutable std::mutex mut;
    int64_t timeToLive{};   // 单帧超时时间，单位为毫秒ms。默认数据永不超时。负值也为永不超时。
                            // 超时的数据会被清理。建议设置的超时时间应该比上游数据的发送时间间隔大。
    size_t capacityLimit{5U};  // 容器容量限制。超过此容量的数据会被清理
    std::deque<T> container;   // 真正放数据的容器
    std::deque<std::chrono::time_point<std::chrono::steady_clock>> times;  // 数据放入此stack的时间点。
};
}  // namespace Adsfi
#endif
