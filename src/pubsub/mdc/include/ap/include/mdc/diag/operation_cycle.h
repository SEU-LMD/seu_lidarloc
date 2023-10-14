/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef MDC_DIAG_OPERATION_CYCLE_H
#define MDC_DIAG_OPERATION_CYCLE_H
#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include "mdc/common/result.h"

namespace mdc {
namespace diag {
namespace internal {
class OperationCycleAgent;
}
class OperationCycle {
public:
    enum class OperationCycleType : uint8_t {
        OPERATION_CYCLE_START = 0x00U,
        OPERATION_CYCLE_END = 0x01U
    };

    explicit OperationCycle(const std::string &specifier);
    virtual ~OperationCycle();
    OperationCycle(OperationCycle&&);
    OperationCycle& operator=(OperationCycle&&);
    OperationCycle(const OperationCycle&) = delete;
    OperationCycle& operator=(const OperationCycle&) = delete;

    // Get current OperationCycle.
    mdc::common::Result<OperationCycleType> GetOperationCycle();

    // Registering a notifier function which is called if the operation cycle is changed
    mdc::common::Result<void> SetNotifier(std::function<void(OperationCycleType)> notifier);

    // Set OperationCycle.
    mdc::common::Result<void> SetOperationCycle(OperationCycleType operationCycle);
private:
    std::string opCycleSpecifier_;
    std::mutex instanceSpecifierMutex_;
    std::shared_ptr<internal::OperationCycleAgent> agent_;
};
}
}
#endif // MDC_DIAG_OPERATION_CYCLE_H