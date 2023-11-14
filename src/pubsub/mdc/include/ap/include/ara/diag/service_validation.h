/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: service_validation.h is designed for application to customize its own validation
 */
#ifndef ARA_SERVICE_VALIDATION_H
#define ARA_SERVICE_VALIDATION_H

#include "ara/core/result.h"
#include "ara/core/future.h"
#include "ara/core/promise.h"
#include "ara/core/vector.h"
#include "ara/core/span.h"
#include "ara/core/instance_specifier.h"
#include "ara/diag/meta_info.h"
#include "ara/diag/diag_error_domain.h"
namespace mdc {
namespace diag {
class ServiceValidationAgent;
}
}

namespace ara {
namespace diag {
class ServiceValidation {
public:
    /**
    * \brief Represents the status of the service processing
    **/
    enum class ConfirmationStatusType : uint8_t {
        kResPosOk = 0x00U,
        kResPosNotOk = 0x01U,
        kResNegOk = 0x02U,
        kResNegNotOk = 0x03U,
        kResPosSuppressed = 0x04U,
        kResNegSuppressed = 0x05U,
        kCanceled = 0x06U,
        kNoProcessingNoResponse = 0x07U
    };

    ServiceValidation(const ara::core::InstanceSpecifier& specifier);
    virtual ~ServiceValidation();

    /**
    * \brief publish serviceValidation service. error result type is ara::diag::DiagOfferErr.
    **/
    ara::core::Result<void> Offer();

    void StopOffer();

    /**
    * \brief validate requestData. error result type is ara::diag::DiagUdsNrcErrc.
    */
    virtual ara::core::Future<void> Validate(ara::core::Span<uint8_t> requestData, MetaInfo metaInfo) = 0;

    /**
    * \brief response confirmation. error result type is ara::diag::DiagUdsNrcErrc.
    */
    virtual ara::core::Future<void> Confirmation(ConfirmationStatusType status, MetaInfo metaInfo) = 0;
protected:
    ServiceValidation(ServiceValidation&&);
    ServiceValidation& operator=(ServiceValidation&&);
    ServiceValidation(const ServiceValidation&) = delete;
    ServiceValidation& operator=(const ServiceValidation&) = delete;
private:
    ara::core::String validationSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<mdc::diag::ServiceValidationAgent> agent_;
};
}
}
#endif  // ARA_SERVICE_VALIDATION_H