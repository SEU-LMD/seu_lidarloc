/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: spin lock implementation
 * Create: 2021-02-02
 */
#ifndef VRTF_SPIN_LOCK_H
#define VRTF_SPIN_LOCK_H
#include <mutex>
namespace vrtf {
namespace vcc {
namespace utils {
class RtfSpinLock {
public:
    RtfSpinLock() = default;
    ~RtfSpinLock() = default;
    void Lock();
    void Unlock();
private:
    // The mutex is used in aos-core and the macro is not impl beacause of the head file is public for user.
    std::mutex mutex_;
};
}
}
}
#endif
