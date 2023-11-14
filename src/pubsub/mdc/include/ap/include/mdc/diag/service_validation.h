/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: service_validation.h is designed for application to customize its own validation
 */
#ifndef MDC_SERVICE_VALIDATION_H
#define MDC_SERVICE_VALIDATION_H
#include <vector>
#include <string>

#include "mdc/common/result.h"
#include "mdc/common/future.h"
#include "mdc/common/promise.h"
#include "mdc/diag/meta_info.h"
#include "mdc/diag/diag_error_domain/diag_error_domain.h"
#include "mdc/diag/diag_error_domain/diag_uds_nrc_error_domain.h"

namespace mdc {
namespace diag {
class ServiceValidationAgent;
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

    ServiceValidation(const std::string& specifier);
    virtual ~ServiceValidation();

    /**
    * \brief publish serviceValidation service. error result type is mdc::diag::DiagOfferErr.
    **/
    common::Result<void> Offer();

    void StopOffer();

    /**
    * \brief validate requestData. error result type is mdc::diag::DiagUdsNrcErrc.
    */
    virtual common::Future<void> Validate(std::vector<uint8_t> requestData, mdc::diag::MetaInfo metaInfo) = 0;

    /**
    * \brief response confirmation. error result type is mdc::diag::DiagUdsNrcErrc.
    */
    virtual common::Future<void> Confirmation(ConfirmationStatusType status, mdc::diag::MetaInfo metaInfo) = 0;
protected:
    ServiceValidation(ServiceValidation&&);
    ServiceValidation& operator=(ServiceValidation&&);
    ServiceValidation(const ServiceValidation&) = delete;
    ServiceValidation& operator=(const ServiceValidation&) = delete;
private:
    std::string mdcValidationSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<ServiceValidationAgent> agent_;
};
}
}
#endif