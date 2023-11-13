/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:This interface is replacing the obsolete DTCInformation, DiagnosticMemory and
 * DiagnosticServer service interfaces.
 * The InstanceSpecifier is only compatible with PortInterface of DiagnosticDTCInformationInterface.
 */
#ifndef MDC_DTC_INFORMATION_H
#define MDC_DTC_INFORMATION_H

#include <string>
#include <functional>
#include <bitset>
#include <cmath>
#include <mutex>
#include "mdc/common/result.h"
namespace mdc {
namespace diag {
namespace internal {
class DtcInformationAgent;
}
class DTCInformation {
public:
    enum class UdsDtcStatusBitType : uint8_t {
        TEST_FAILED = 0x01U,
        TEST_FAILED_THIS_OPERATION_CYCLE = 0x02U,
        PENDING_DTC = 0x04U,
        CONFIRMED_DTC = 0x08U,
        TEST_NOT_COMPLETED_SINCE_LAST_CLEAR = 0x10U,
        TEST_FAILED_SINCE_LAST_CLEAR = 0x20U,
        TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE = 0x40U,
        WARNING_INDICATOR_REQUESTED = 0x80U
    };
    struct UdsDtcStatusByteType {
        uint8_t dtcStatu;
        uint8_t GetBitType (UdsDtcStatusBitType bitType)
        {
            return (dtcStatu & static_cast<uint8_t>(bitType)) >>
                static_cast<uint8_t>(std::log2(static_cast<uint8_t>(bitType)));
        }
    };

    using DtcInformationNotifierType = std::function<void(std::uint32_t dtc,
            UdsDtcStatusByteType udsStatusByteOld, UdsDtcStatusByteType udsStatusByteNew)>;
    DTCInformation(const std::string &specifier);
    virtual ~DTCInformation();
    DTCInformation(const DTCInformation&) = delete;
    DTCInformation& operator=(const DTCInformation&) = delete;
    DTCInformation(DTCInformation&&);
    DTCInformation& operator=(DTCInformation&&);
    mdc::common::Result<void> SetDtcStatusChangedNotifier(DtcInformationNotifierType notifier);
private:
    std::string dtcInfoSpecifier_;
    std::mutex instanceSpecifierMutex_;
    std::shared_ptr<internal::DtcInformationAgent> agent_;
};
}
}
#endif // MDC_DTC_INFORMATION_H
