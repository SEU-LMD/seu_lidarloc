/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:
 */
#ifndef MDC_DIAG_SECURITY_ACCESS_H
#define MDC_DIAG_SECURITY_ACCESS_H
#include <vector>
#include <string>
#include <memory>
#include "mdc/common/result.h"
#include "mdc/common/future.h"
#include "mdc/common/promise.h"
#include "mdc/diag/meta_info.h"
#include "mdc/diag/cancellation_handler.h"
#include "mdc/diag/diag_error_domain/diag_error_domain.h"
#include "mdc/diag/diag_error_domain/diag_uds_nrc_error_domain.h"

namespace mdc {
namespace diag {
class SecurityAccessAgent;
class SecurityAccess {
public:
    /**
    * \brief Represents the status of the key compare.
    **/
    enum class KeyCompareResultType : uint8_t {
        kKeyValid = 0x00U,
        kKeyInvalid = 0x01U,
    };

    explicit SecurityAccess(const std::string& specifier);
    virtual ~SecurityAccess() = default;

    virtual common::Future<std::vector<std::uint8_t>> GetSeed(
        const std::vector<std::uint8_t>& securityAccessDataRecord,
        mdc::diag::MetaInfo metaInfo, mdc::diag::CancellationHandler cancellationHandler) = 0;

    virtual common::Future<KeyCompareResultType> CompareKey(const std::vector<std::uint8_t>& key,
        mdc::diag::MetaInfo metaInfo, mdc::diag::CancellationHandler cancellationHandler) = 0;

    common::Result<void> Offer();
    void StopOffer();
protected:
    SecurityAccess(const SecurityAccess&) = default;
    SecurityAccess& operator=(const SecurityAccess&) = default;
    SecurityAccess(SecurityAccess&&) = default;
    SecurityAccess& operator=(SecurityAccess&&) = default;

    std::string securityAccessSpecifier_;
    std::shared_ptr<SecurityAccessAgent> agent_;
};
}
}
#endif // MDC_DIAG_SECURITY_ACCESS_H