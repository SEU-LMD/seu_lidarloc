/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Class to implement operations on diagnostic Monitor interface.
 */
#ifndef MDC_DIAG_MONITOR_H
#define MDC_DIAG_MONITOR_H
#include <vector>
#include <string>
#include <functional>
#include "mdc/common/result.h"

namespace mdc {
namespace diag {
namespace internal {
class MonitorAgent;
}
class Monitor final {
public:
    enum class InitMonitorReason : uint8_t {
        CLEAR = 0U,
        RESTART = 1U,
        REENABLED = 2U,
    };

    enum class MonitorAction : uint8_t {
        PASSED = 0U,
        FAILED = 1U,
        PREPASSED = 2U,
        PREFAILED = 3U,
        FDC_THRESHOLD_REACHED = 4U,
        RESET_TEST_FAILED = 5U,
        FREEZE_DEBOUNCING = 6U,
        RESET_DEBOUNCING = 7U,
        PRESTORE = 8U,
        CLEAR_PRESTORE = 9U
    };

    struct DidInfo {
        uint16_t didId;
        std::vector<uint8_t> didValue;
    };

    explicit Monitor(const std::string &specifier,
                    std::function<void(InitMonitorReason)> initMonitor = nullptr,
                    std::function<std::int8_t()> getFaultDetectionCounter = nullptr);
    virtual ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;
    Monitor(Monitor&&);
    Monitor& operator=(Monitor&&);
    bool operator==(const Monitor&) const;
    bool operator!=(const Monitor&) const;
    virtual mdc::common::Result<void> ReportMonitorAction(const MonitorAction &action);
    virtual void ReportMonitorAction(const MonitorAction &action,
                                    const std::vector<Monitor::DidInfo> &didInfos,
                                    std::function<void(mdc::common::Result<void>&)> callback);
private:
    std::string mdcMonitorSpecifier_;
    bool isInitialized_;
    std::mutex monitorMutex_;
    std::shared_ptr<internal::MonitorAgent> agent_;
};
}
}
#endif // MDC_DIAG_MONITOR_H