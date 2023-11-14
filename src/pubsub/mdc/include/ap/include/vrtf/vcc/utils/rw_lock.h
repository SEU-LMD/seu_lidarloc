/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: RWLock implemented by atomic, C++ 11 is required
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_UTILS_RWLOCK_H
#define VRTF_VCC_UTILS_RWLOCK_H
#include <cstdlib>
#include <atomic>
#include <thread>
#include "raii.h"

namespace vrtf {
namespace vcc {
namespace utils {
class RWLock {
/* positive value means shared read status */
const int WRITE_LOCK_STATUS = -1;
const int FREE_STATUS = 0;

public:
    RWLock(const RWLock &) = delete;
    RWLock& operator =(const RWLock &) = delete;
    RWLock (bool WriteFirst = false);
    virtual ~RWLock() = default;
    int ReadLock();
    int ReadUnlock();
    int WriteLock();
    int WriteUnlock();

    Raii ReadGuard() const noexcept
    {
        return MakeRaii(*this, &RWLock::ReadUnlock, &RWLock::ReadLock);
    }
    Raii WriteGuard() const noexcept
    {
        return MakeRaii(*this, &RWLock::WriteUnlock, &RWLock::WriteLock);
    }

private:
    static const std::thread::id NULL_THEAD;
    const bool writeFirst;
    std::thread::id writeThreadId;
    std::atomic_int lockCount;
    std::atomic_uint writeWaitCount;
};
}
}
}
#endif
