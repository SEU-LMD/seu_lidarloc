/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: lock free queue head file.
 * Create: 2020-06-12
 */
#ifndef VRTF_LOCKFREEQUEUE_H
#define VRTF_LOCKFREEQUEUE_H
namespace vrtf {
namespace vcc {
namespace utils {
template <typename NodeType>
class LockFreeQueue {
public:
    explicit LockFreeQueue(uint32_t maxSize)
        : queueSize_(0), queueMaxSize_(maxSize + 1), writeIndex_(0), readIndex_(0), maxReadIndex_(0)
    {
        queue_.resize(maxSize + 1);
    }
    LockFreeQueue() = delete;
    ~LockFreeQueue() = default;
    uint32_t GetSize()
    {
        return queueSize_;
    }

    bool Push(const NodeType &node)
    {
        uint32_t curWriteIndex;
        uint32_t curReadIndex;

        do {
            curWriteIndex = writeIndex_;
            curReadIndex = readIndex_;
            if (CalcIndex(curWriteIndex + 1) == CalcIndex(curReadIndex)) {
                /* the queue is full. */
                return false;
            }
        } while (!__sync_bool_compare_and_swap(&writeIndex_, static_cast<int>(curWriteIndex),
            static_cast<int>(curWriteIndex + 1)));

        queue_[CalcIndex(curWriteIndex)] = node;

        while (!__sync_bool_compare_and_swap(&maxReadIndex_, static_cast<int>(curWriteIndex),
            static_cast<int>(curWriteIndex + 1))) {
            sched_yield();
        }
        __sync_fetch_and_add (&queueSize_, 1);
        return true;
    }

    bool Pop(NodeType &node)
    {
        uint32_t curReadIndex;
        uint32_t curMaxReadIndex;
        do {
            curReadIndex = readIndex_;
            curMaxReadIndex = maxReadIndex_;
            if (CalcIndex(curReadIndex) == CalcIndex(curMaxReadIndex)) {
                /* queue is empty. */
                return false;
            }
            node = queue_[CalcIndex(curReadIndex)];
            queue_[CalcIndex(curReadIndex)] = typename std::decay<NodeType>::type();
            if (__sync_bool_compare_and_swap(&readIndex_, static_cast<int>(curReadIndex),
                static_cast<int>(curReadIndex + 1))) {
                __sync_fetch_and_sub(&queueSize_, 1);
                return true;
            }
        } while (true);
    }

    bool GetFront(NodeType &node)
    {
        uint32_t curReadIndex;
        uint32_t curMaxReadIndex;
        do {
            curReadIndex = readIndex_;
            curMaxReadIndex = maxReadIndex_;
            if (CalcIndex(curReadIndex) == CalcIndex(curMaxReadIndex)) {
                /* queue is empty. */
                return false;
            }
            node = queue_[CalcIndex(curReadIndex)];
            if (__sync_bool_compare_and_swap(&readIndex_, curReadIndex, readIndex_)) {
                return true;
            }
        } while (true);
    }

    bool At(uint32_t pos, NodeType &node)
    {
        uint32_t curReadIndex;
        uint32_t curMaxReadIndex;
        do {
            curReadIndex = readIndex_;
            curMaxReadIndex = maxReadIndex_;
            if (CalcIndex(curReadIndex) == CalcIndex(curMaxReadIndex)) {
                /* queue is empty. */
                return false;
            }
            if (pos >= queueSize_) {
                /* exceed queue size. */
                return false;
            }
            node = queue_[CalcIndex(curReadIndex + pos)];
            if (__sync_bool_compare_and_swap(&readIndex_, curReadIndex, readIndex_)) {
                return true;
            }
        } while (true);
    }

    bool Empty()
    {
        return (GetSize() == 0);
    }

private:
    inline uint32_t CalcIndex(uint32_t num)
    {
        return num % queueMaxSize_;
    }
    std::vector<NodeType> queue_;
    uint32_t queueSize_;
    uint32_t queueMaxSize_;
    uint32_t writeIndex_;
    uint32_t readIndex_;
    uint32_t maxReadIndex_;
};
}
}
}
#endif // VRTF_LOCKFREEQUEUE_H
