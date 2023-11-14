/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:This interface is replacing the obsolete DTCInformation, DiagnosticMemory and
 * DiagnosticServer service interfaces.
 * The InstanceSpecifier is only compatible with PortInterface of DiagnosticDTCInformationInterface.
 */
#ifndef ARA_DIAG_DTC_INFORMATION_H
#define ARA_DIAG_DTC_INFORMATION_H
#include <functional>
#include <bitset>
#include <cmath>
#include <mutex>
#include "ara/core/result.h"
#include "ara/core/instance_specifier.h"
#include "ara/core/string.h"

namespace mdc {
namespace diag {
namespace internal {
class DtcInformationAgent;
}
}
}
namespace ara {
namespace diag {
class DTCInformation {
public:
    enum class UdsDtcStatusBitType : uint8_t {
        kTestFailed = 0x01U,
        kTestFailedThisOperationCycle = 0x02U,
        kPendingDTC = 0x04U,
        kConfirmedDTC = 0x08U,
        kTestNotCompletedSinceLastClear = 0x10U,
        kTestFailedSinceLastClear = 0x20U,
        kTestNotCompletedThisOperationCycle = 0x40U,
        kWarningIndicatorRequested = 0x80U
    };

    struct UdsDtcStatusByteType {
        uint8_t dtcStatu;
        uint8_t GetBitType (UdsDtcStatusBitType bitType)
        {
            return (dtcStatu & static_cast<uint8_t>(bitType)) >>
                static_cast<uint8_t>(std::log2(static_cast<uint8_t>(bitType)));
        }
    };
    using DtcInformationNotifierType= std::function<void(std::uint32_t dtc,
            UdsDtcStatusByteType udsStatusByteOld, UdsDtcStatusByteType udsStatusByteNew)>;
    DTCInformation(const ara::core::InstanceSpecifier &specifier);
    virtual ~DTCInformation();
    DTCInformation(const DTCInformation&) = delete;
    DTCInformation& operator=(const DTCInformation&) = delete;
    DTCInformation(DTCInformation&&);
    DTCInformation& operator=(DTCInformation&&);
    ara::core::Result<void> SetDtcStatusChangedNotifier(DtcInformationNotifierType notifier);
private:
    ara::core::String instanceSpecifier_;
    std::mutex instanceSpecifierMutex_;
    std::shared_ptr<mdc::diag::internal::DtcInformationAgent> agent_;
};
}
}
#endif // ARA_DIAG_DTC_INFORMATION_H
