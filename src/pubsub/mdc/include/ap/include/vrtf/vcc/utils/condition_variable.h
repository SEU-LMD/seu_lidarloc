/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: define thread lock
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_UTILS_CONDITIONVARIABLE_H
#define VRTF_VCC_UTILS_CONDITIONVARIABLE_H
#include <mutex>
#include <condition_variable>
#include "ara/hwcommon/log/log.h"
namespace vrtf {
namespace vcc {
namespace utils {
class ConditionVariable {
public:
    ConditionVariable()
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~ConditionVariable() = default;
    void Wait();
    void NotifyAll();
private:
    std::condition_variable cond_;
    std::mutex   mutex_;
    bool needWait_ = true;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
#endif
