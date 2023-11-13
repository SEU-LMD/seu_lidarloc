/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef MDC_DIAG_GENERIC_DATA_IDENTIFIER_H
#define MDC_DIAG_GENERIC_DATA_IDENTIFIER_H
#include <vector>
#include <string>
#include "mdc/common/promise.h"
#include "mdc/diag/meta_info.h"
#include "mdc/common/result.h"
#include "mdc/common/future.h"
#include "mdc/diag/cancellation_handler.h"
#include "mdc/diag/diag_error_domain/diag_error_domain.h"
#include "mdc/diag/diag_error_domain/diag_uds_nrc_error_domain.h"

namespace mdc {
namespace diag {
class DataIdentifierAgent;
class GenericDataIdentifier {
public:
    using OperationOutput = std::vector<uint8_t>;
    explicit GenericDataIdentifier(const std::string& specifier);
    virtual ~GenericDataIdentifier();
    common::Result<void> Offer();
    void StopOffer();

    virtual common::Future<OperationOutput> Read(std::uint16_t dataIdentifier, mdc::diag::MetaInfo metaInfo,
        mdc::diag::CancellationHandler cancellationHandler) = 0;

    virtual common::Future<void> Write(std::uint16_t dataIdentifier, const std::vector<uint8_t>& requestData,
        mdc::diag::MetaInfo metaInfo, mdc::diag::CancellationHandler cancellationHandler) = 0;
protected:
    GenericDataIdentifier(const GenericDataIdentifier&) = delete;
    GenericDataIdentifier& operator=(const GenericDataIdentifier&) = delete;
    GenericDataIdentifier(GenericDataIdentifier&&);
    GenericDataIdentifier& operator=(GenericDataIdentifier&&);
private:
    std::string didSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<DataIdentifierAgent> agent_;
};
}
}
#endif // MDC_DIAG_GENERIC_DATA_IDENTIFIER_H