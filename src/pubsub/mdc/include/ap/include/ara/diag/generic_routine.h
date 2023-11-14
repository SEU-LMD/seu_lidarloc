/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:generic_routine.h is designed for application to customize its own routine
 */
#ifndef ARA_DIAG_GENERIC_ROUTINE_H
#define ARA_DIAG_GENERIC_ROUTINE_H
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/span.h"
#include "ara/core/future.h"
#include "ara/core/result.h"
#include "ara/core/instance_specifier.h"
#include "ara/diag/meta_info.h"
#include "ara/diag/cancellation_handler.h"
#include "ara/diag/diag_error_domain.h"

namespace mdc {
namespace diag {
class RoutineAgent;
}
}

namespace ara {
namespace diag {
class GenericRoutine {
public:
    using OperationOutput = ara::core::Vector<uint8_t>;
    explicit GenericRoutine(const ara::core::InstanceSpecifier& specifier);
    virtual ~GenericRoutine();
    ara::core::Result<void> Offer();
    void StopOffer();
    virtual ara::core::Future<OperationOutput> Start(uint16_t routineId, ara::core::Span<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
    virtual ara::core::Future<OperationOutput> Stop(uint16_t routineId, ara::core::Span<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
    virtual ara::core::Future<OperationOutput> RequestResult(uint16_t routineId, ara::core::Span<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
protected:
    GenericRoutine(const GenericRoutine&) = delete;
    GenericRoutine& operator=(const GenericRoutine&) = delete;
    GenericRoutine(GenericRoutine&&);
    GenericRoutine& operator=(GenericRoutine&&);
private:
    ara::core::String routineSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<mdc::diag::RoutineAgent> agent_;
};
}
}
#endif