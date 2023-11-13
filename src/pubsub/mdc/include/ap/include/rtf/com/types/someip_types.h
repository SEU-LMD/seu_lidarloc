/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */
#ifndef RTF_COM_TYPES_SOMEIP_TYPES_H_
#define RTF_COM_TYPES_SOMEIP_TYPES_H_

#include <cstdint>
#include <vrtf/driver/someip/someip_driver_types.h>
namespace rtf {
namespace com {
namespace someip {
using ServiceId    = uint16_t;
using InstanceId   = uint16_t;
using EventId      = uint16_t;
using EventGroupId = uint16_t;
using MethodId     = uint16_t;
using Port         = uint16_t;
using MajorVersion = uint8_t;
using MinorVersion = uint32_t;
using SdInfo       = vrtf::driver::someip::SomeipServiceDiscoveryInfo;
using EventInfo    = vrtf::driver::someip::SomeipEventInfo;
using MethodInfo   = vrtf::driver::someip::SomeipMethodInfo;
namespace serialize {
using WireType     = vrtf::serialize::WireType;
using ByteOrder    = vrtf::serialize::ByteOrder;
enum class StaticLengthField: uint8_t {
    ONE_BYTE,
    TWO_BYTES,
    FOUR_BYTES
};
}

struct ComposePacketInfo {
    std::string udpTriggerMode       = "trigger_always";
    int32_t composePacketCachingTime = 1000;
};
// Diagnosis types
using DiagnosisCounterType        = vrtf::driver::someip::DiagnosisCounterType;
using FaultsReportType            = vrtf::driver::someip::FaultsReportType;
using FaultsDiagnosisHandler      = vrtf::driver::someip::FaultsDiagnosisHandler;
using FaultsDiagnosisCallbackType = vrtf::driver::someip::FaultsDiagnosisCallbackType;
using ResetDiagnosisCounterType   = vrtf::driver::someip::ResetDiagnosisCounterType;
} // namespace someip
} // namespace com
} // namespace rtf
#endif // RTF_COM_TYPES_SOMEIP_TYPES_H_
