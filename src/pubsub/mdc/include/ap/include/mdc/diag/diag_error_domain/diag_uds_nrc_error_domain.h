/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: uds error domain
 */
#ifndef MDC_DIAG_UDS_ERROR_DOMAIN_H
#define MDC_DIAG_UDS_ERROR_DOMAIN_H
#include <cstdint>
#include "mdc/common/error_code.h"
#include "mdc/common/error_domain.h"
namespace mdc {
namespace diag {
enum class DiagUdsNrcErrc : mdc::common::ErrorDomain::CodeType {
    kGeneralReject = 0x10,
    kServiceNotSupported = 0x11,
    kSubfunctionNotSupported = 0x12,
    kIncorrectMessageLengthOrInvalidFormat = 0x13,
    kResponseTooLong = 0x14,
    kBusyRepeatRequest = 0x21,
    kConditionsNotCorrect = 0x22,
    kRequestSequenceError = 0x24,
    kNoResponseFromSubnetComponent = 0x25,
    kFailurePreventsExecutionOfRequestedAction = 0x26,
    kRequestOutOfRange = 0x31,
    kSecurityAccessDenied = 0x33,
    kInvalidKey = 0x35,
    kExceedNumberOfAttempts = 0x36,
    kRequiredTimeDelayNotExpired = 0x37,
    kUploadDownloadNotAccepted = 0x70,
    kTransferDataSuspended = 0x71,
    kGeneralProgrammingFailure = 0x72,
    kWrongBlockSequenceCounter = 0x73,
    kSubFunctionNotSupportedInActiveSession = 0x7E,
    kServiceNotSupportedInActiveSession = 0x7F,
    kRpmTooHigh = 0x81,
    kRpmTooLow = 0x82,
    kEngineIsRunning = 0x83,
    kEngineIsNotRunning = 0x84,
    kEngineRunTimeTooLow = 0x85,
    kTemperatureTooHigh = 0x86,
    kTemperatureTooLow = 0x87,
    kVehicleSpeedTooHigh = 0x88,
    kVehicleSpeedTooLow = 0x89,
    kThrottlePedalTooHigh = 0x8A,
    kThrottlePedalTooLow = 0x8B,
    kTransmissionRangeNotInNeutral = 0x8C,
    kTransmissionRangeNotInGear = 0x8D,
    kBrakeSwitchNotClosed = 0x8F,
    kShifterLeverNotInPark = 0x90,
    kTorqueConverterClutchLocked = 0x91,
    kVoltageTooHigh = 0x92,
    kVoltageTooLow = 0x93,
    kNoProcessingNoResponse = 0xFF
};
static const std::map<DiagUdsNrcErrc, char const *> g_nrcToString = {
    {DiagUdsNrcErrc::kGeneralReject, "General reject"},
    {DiagUdsNrcErrc::kServiceNotSupported, "Service not supported"},
    {DiagUdsNrcErrc::kSubfunctionNotSupported, "SubFunction not supported"},
    {DiagUdsNrcErrc::kIncorrectMessageLengthOrInvalidFormat, "Incorrect message length or invalid format"},
    {DiagUdsNrcErrc::kResponseTooLong, "Response too long"},
    {DiagUdsNrcErrc::kBusyRepeatRequest, "Busy repeat request"},
    {DiagUdsNrcErrc::kConditionsNotCorrect, "Condition not correct"},
    {DiagUdsNrcErrc::kRequestSequenceError, "Request sequence error"},
    {DiagUdsNrcErrc::kNoResponseFromSubnetComponent, "No response from subnet component"},
    {DiagUdsNrcErrc::kFailurePreventsExecutionOfRequestedAction, "Failure prevents execution of requested action"},
    {DiagUdsNrcErrc::kRequestOutOfRange, "Request out of range"},
    {DiagUdsNrcErrc::kSecurityAccessDenied, "Security access denied"},
    {DiagUdsNrcErrc::kInvalidKey, "Invalid Key"},
    {DiagUdsNrcErrc::kExceedNumberOfAttempts, "Exceed number of attempts"},
    {DiagUdsNrcErrc::kRequiredTimeDelayNotExpired, "Required time delay not expired"},
    {DiagUdsNrcErrc::kUploadDownloadNotAccepted, "UploadDownload not accepted"},
    {DiagUdsNrcErrc::kTransferDataSuspended, "Transfer data suspended"},
    {DiagUdsNrcErrc::kGeneralProgrammingFailure, "General programming failure"},
    {DiagUdsNrcErrc::kWrongBlockSequenceCounter, "Wrong block sequence counter"},
    {DiagUdsNrcErrc::kSubFunctionNotSupportedInActiveSession, "SubFunction not supported inactive session"},
    {DiagUdsNrcErrc::kServiceNotSupportedInActiveSession, "Service not supported in active session"},
    {DiagUdsNrcErrc::kRpmTooHigh, "Rpm too high"},
    {DiagUdsNrcErrc::kRpmTooLow, "Rpm too low"},
    {DiagUdsNrcErrc::kEngineIsRunning, "Engine is running"},
    {DiagUdsNrcErrc::kEngineIsNotRunning, "Engine is not running"},
    {DiagUdsNrcErrc::kEngineRunTimeTooLow, "Engine run time too low"},
    {DiagUdsNrcErrc::kTemperatureTooHigh, "Temperature too high"},
    {DiagUdsNrcErrc::kTemperatureTooLow, "Temperature too low"},
    {DiagUdsNrcErrc::kVehicleSpeedTooHigh, "Vehicle speed too high"},
    {DiagUdsNrcErrc::kVehicleSpeedTooLow, "Vehicle speed too low"},
    {DiagUdsNrcErrc::kThrottlePedalTooHigh, "Throttle pedal too high"},
    {DiagUdsNrcErrc::kThrottlePedalTooLow, "Throttle pedal too low"},
    {DiagUdsNrcErrc::kTransmissionRangeNotInNeutral, "Transmission range not in neutral"},
    {DiagUdsNrcErrc::kTransmissionRangeNotInGear, "Transmission range not in gear"},
    {DiagUdsNrcErrc::kBrakeSwitchNotClosed, "Brake switch not closed"},
    {DiagUdsNrcErrc::kShifterLeverNotInPark, "Shifter lever not in park"},
    {DiagUdsNrcErrc::kTorqueConverterClutchLocked, "Torque converter clutch locked"},
    {DiagUdsNrcErrc::kVoltageTooHigh, "Voltage too high"},
    {DiagUdsNrcErrc::kVoltageTooLow, "Voltage too low"},
    {DiagUdsNrcErrc::kNoProcessingNoResponse, "No processing no response"}
};

/*
[SWS_DM_00547] Definition UDS NRC ara::diag errors
UDS NRC ara::diag errors shall be defined in the error domain ara::diag::DiagUdsNrcErrorDomain
*/
class DiagUdsNrcErrorDomain final : public mdc::common::ErrorDomain {
public:
    constexpr DiagUdsNrcErrorDomain() noexcept
        : mdc::common::ErrorDomain(kId)
    {}
    ~DiagUdsNrcErrorDomain() = default;

    char const *Name() const noexcept override
    {
        return "DiagUds";
    }
    char const *Message(const mdc::common::ErrorDomain::CodeType& errorCode) const noexcept override
    {
        const auto it = g_nrcToString.find(static_cast<DiagUdsNrcErrc>(errorCode));
        if (it == g_nrcToString.end()) {
            return "Unknow error type";
        }
        return g_nrcToString.at(static_cast<DiagUdsNrcErrc>(errorCode));
    }

private:
    constexpr static mdc::common::ErrorDomain::IdType kId = 0x80000000000000FDU; // need to design
};

namespace internal {
constexpr DiagUdsNrcErrorDomain g_diagUdsErrorDomain;
}

constexpr mdc::common::ErrorDomain const& GetDiagUdsErrorDomain()
{
    return internal::g_diagUdsErrorDomain;
}

constexpr mdc::common::ErrorCode MakeErrorCode(const DiagUdsNrcErrc& udsNrcErrorCode,
    const mdc::common::ErrorDomain::SupportDataType& supportData = mdc::common::ErrorDomain::SupportDataType()) noexcept
{
    return mdc::common::ErrorCode(static_cast<mdc::common::ErrorDomain::CodeType>(udsNrcErrorCode),
        GetDiagUdsErrorDomain(), supportData);
}
}
}
#endif
