/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef ARA_DIAG_OPERATION_CYCLE_H
#define ARA_DIAG_OPERATION_CYCLE_H
#include <functional>
#include <mutex>
#include "ara/core/result.h"
#include "ara/core/instance_specifier.h"

namespace mdc {
namespace diag {
namespace internal {
class OperationCycleAgent;
}
}
}

namespace ara {
namespace diag {
class OperationCycle {
public:
    enum class OperationCycleType : uint8_t {
        kOperationCycleStart = 0x00U,
        kOperationCycleEnd = 0x01U
    };

    explicit OperationCycle(const ara::core::InstanceSpecifier &specifier);
    virtual ~OperationCycle();
    OperationCycle(OperationCycle&&);
    OperationCycle& operator=(OperationCycle&&);
    OperationCycle(const OperationCycle&) = delete;
    OperationCycle& operator=(const OperationCycle&) = delete;
    // Get current OperationCycle.
    ara::core::Result<OperationCycleType> GetOperationCycle();

    // Registering a notifier function which is called if the operation cycle is changed
    ara::core::Result<void> SetNotifier(std::function< void(OperationCycleType)> notifier);

    // Set OperationCycle.
    ara::core::Result<void> SetOperationCycle(OperationCycleType operationCycle);
private:
    ara::core::String operationCycleSpecifier_;
    std::mutex instanceSpecifierMutex_;
    std::shared_ptr<mdc::diag::internal::OperationCycleAgent> agent_;
};
}
}
#endif // ARA_DIAG_OPERATION_CYCLE_H