/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: 队列模块，提供异步队列基本功能.
 */
#ifndef HAF_CORE_QUEUE_H
#define HAF_CORE_QUEUE_H
#include <mutex>
#include <queue>
#include "core/status.h"

namespace Adsfi {
    template <typename T> class HafQueue {
    public:
        HafQueue() {}
        ~HafQueue() {}

        HafStatus EnQueue(T item);
        HafStatus DeQueue(T &item);
        size_t Size() const;
        bool Empty() const;

    private:
        std::mutex mutex_;
        std::queue<T> queue_;
    };

    template <typename T> HafStatus HafQueue<T>::EnQueue(T item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.emplace(item);
        return HAF_SUCCESS;
    }

    template <typename T> HafStatus HafQueue<T>::DeQueue(T &item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return HAF_NULL;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return HAF_SUCCESS;
    }

    template <typename T> size_t HafQueue<T>::Size() const
    {
        return queue_.size();
    }

    template <typename T> bool HafQueue<T>::Empty() const
    {
        return queue_.empty();
    }
}
#endif
