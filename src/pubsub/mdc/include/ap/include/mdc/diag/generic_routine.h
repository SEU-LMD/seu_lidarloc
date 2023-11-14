/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:generic_routine.h is designed for application to customize its own routine
 */
#ifndef MDC_GENERIC_ROUTINE_H
#define MDC_GENERIC_ROUTINE_H
#include <vector>
#include <string>

#include "mdc/common/future.h"
#include "mdc/common/promise.h"
#include "mdc/diag/meta_info.h"
#include "mdc/common/result.h"
#include "mdc/diag/cancellation_handler.h"
#include "mdc/diag/diag_error_domain/diag_error_domain.h"
#include "mdc/diag/diag_error_domain/diag_uds_nrc_error_domain.h"

namespace mdc {
namespace diag {
class RoutineAgent;
class GenericRoutine {
public:
    using OperationOutput = std::vector<uint8_t>;
    explicit GenericRoutine(const std::string& specifier);
    virtual ~GenericRoutine();
    common::Result<void> Offer();
    void StopOffer();
    virtual common::Future<OperationOutput> Start(uint16_t routineId, std::vector<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
    virtual common::Future<OperationOutput> Stop(uint16_t routineId, std::vector<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
    virtual common::Future<OperationOutput> RequestResult(uint16_t routineId, std::vector<uint8_t> requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
protected:
    GenericRoutine(const GenericRoutine&) = delete;
    GenericRoutine& operator=(const GenericRoutine&) = delete;
    GenericRoutine(GenericRoutine&&);
    GenericRoutine& operator=(GenericRoutine&&);
private:
    std::string mdcRoutineSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<RoutineAgent> agent_;
};
}
}
#endif