/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */

#ifndef SOMEIP_TYPES_H
#define SOMEIP_TYPES_H

#include <cstdint>
#include <array>
#include <map>

namespace Someip {
using MessageID = uint32_t;
using ServiceID = uint16_t;
using MethodID = uint16_t;
using EventID = uint16_t;

using InstanceID = uint16_t;
using EventgroupID = uint16_t;

using MajorVersion = uint8_t;
using MinorVersion = uint32_t;

using TTL = uint32_t;

using RequestID = uint32_t;
using ClientID = uint16_t;
using SessionID = uint16_t;

using ProtocolVersion = uint8_t;
using InterfaceVersion = uint8_t;

using Ipv4Address = std::array<uint8_t, 4>;
using Ipv6Address = std::array<uint8_t, 16>;

using SubscriptionID = std::uint16_t;

using AvailableMap = std::map<ServiceID, std::map<InstanceID, std::map<MajorVersion, MinorVersion>>>;

struct MethodTimeoutBody {
    ServiceID service;
    MethodID method;
    SessionID oldSession;
    SessionID newSession;
};
}

#endif
