/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:cancellation_handler is designed for handle external service cancellation
 */
#ifndef MDC_DIAG_CANCELLATION_HANDLER
#define MDC_DIAG_CANCELLATION_HANDLER
#include <functional>
namespace mdc {
namespace diag {
class CancellationHandler final {
public:
    CancellationHandler() = delete;
    ~CancellationHandler() = default;
    explicit CancellationHandler(const uint64_t serialNumber) : serialNumber_(serialNumber) {};
    CancellationHandler(CancellationHandler&) = delete;
    CancellationHandler(CancellationHandler&&) = default;
    CancellationHandler& operator=(CancellationHandler&&) = default;
    CancellationHandler& operator=(CancellationHandler&) = delete;
    bool IsCanceled() const;
    void SetNotifier(std::function<void()> notifier);
    std::function<void()> GetCallback() const
    {
        return callback_;
    }

private:
    std::function<void()> callback_;
    bool isCanceled = false;
    uint64_t serialNumber_;
};
}
}
#endif