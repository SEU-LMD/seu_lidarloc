/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Class to implement operations on diagnostic Monitor interface.
 */
#ifndef ARA_DIAG_MONITOR_H
#define ARA_DIAG_MONITOR_H
#include <functional>
#include <mutex>
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/result.h"
#include "ara/core/instance_specifier.h"

namespace mdc {
namespace diag {
namespace internal {
class MonitorAgent;
}
}
}
namespace ara {
namespace diag {
class Monitor final {
public:
    enum class InitMonitorReason : uint8_t {
        kClear = 0,
        kRestart = 1,
        kReenabled = 2,
    };

    enum class MonitorAction : uint8_t {
        kPassed = 0,
        kFailed = 1,
        kPrepassed = 2,
        kPrefailed = 3,
        kFdcThresholdReached = 4,
        kResetTestFailed = 5,
        kFreezeDebouncing = 6,
        kResetDebouncing = 7,
        kPrestore = 8,
        kClearPrestore = 9
    };

    Monitor(const ara::core::InstanceSpecifier &specifier,
                    std::function<void(InitMonitorReason)> initMonitor = nullptr,
                    std::function<int8_t()> getFaultDetectionCounter = nullptr);
    virtual ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;
    Monitor(Monitor&&);
    Monitor& operator=(Monitor&&);
    bool operator==(const Monitor&) const;
    bool operator!=(const Monitor&) const;
    ara::core::Result<void> ReportMonitorAction(const MonitorAction &action);
private:
    ara::core::String monitorSpecifier_;
    bool isInitialized_;
    std::mutex monitorMutex_;
    std::shared_ptr<mdc::diag::internal::MonitorAgent> agent_;
};
}
}
#endif // ARA_DIAG_MONITOR_H_