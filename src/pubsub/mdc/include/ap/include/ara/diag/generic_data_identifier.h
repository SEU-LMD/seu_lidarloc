/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef ARA_DIAG_GENERIC_DATA_IDENTIFIER_H
#define ARA_DIAG_GENERIC_DATA_IDENTIFIER_H
#include <mutex>
#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/core/future.h"
#include "ara/core/span.h"
#include "ara/core/instance_specifier.h"
#include "meta_info.h"
#include "cancellation_handler.h"

namespace mdc {
namespace diag {
class DataIdentifierAgent;
}
}

namespace ara {
namespace diag {
class GenericDataIdentifier {
public:
    using OperationOutput = ara::core::Vector<uint8_t>;
    explicit GenericDataIdentifier(const ara::core::InstanceSpecifier& specifier);
    virtual ~GenericDataIdentifier();
    ara::core::Result<void> Offer();
    void StopOffer();

    virtual ara::core::Future<OperationOutput> Read(uint16_t dataIdentifier, MetaInfo metaInfo,
        CancellationHandler cancellationHandler) = 0;

    virtual ara::core::Future<void> Write(uint16_t dataIdentifier, const ara::core::Span<uint8_t>& requestData,
        MetaInfo metaInfo, CancellationHandler cancellationHandler) = 0;
protected:
    GenericDataIdentifier(const GenericDataIdentifier&) = delete;
    GenericDataIdentifier& operator=(const GenericDataIdentifier&) = delete;
    GenericDataIdentifier(GenericDataIdentifier&&);
    GenericDataIdentifier& operator=(GenericDataIdentifier&&);
private:
    ara::core::String dataIdentifierSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<mdc::diag::DataIdentifierAgent> agent_;
};
}
}
#endif // ARA_DIAG_GENERIC_DATA_IDENTIFIER_H