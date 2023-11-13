/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 */
// --------------------------------------------------------------------------
// |              _    _ _______     .----.      _____         _____        |
// |         /\  | |  | |__   __|  .  ____ .    / ____|  /\   |  __ \       |
// |        /  \ | |  | |  | |    .  / __ \ .  | (___   /  \  | |__) |      |
// |       / /\ \| |  | |  | |   .  / / / / v   \___ \ / /\ \ |  _  /       |
// |      / /__\ \ |__| |  | |   . / /_/ /  .   ____) / /__\ \| | \ \       |
// |     /________\____/   |_|   ^ \____/  .   |_____/________\_|  \_\      |
// |                              . _ _  .                                  |
// --------------------------------------------------------------------------

/// @file
/// @brief Common Types for the UDS TransportLayer C++ Interfaces

#ifndef APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_TYPES_H
#define APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_TYPES_H

#include <cstdint>
#include "ara/core/string.h"
#include "ara/core/vector.h"

namespace ara {
namespace diag {
namespace uds_transport {
// @brief The type of UDS message payloads.
// @uptrace{SWS_DM_00338}
using ByteVector = ara::core::Vector<uint8_t>;

// @brief The identifier of a logical (network) channel, over which UDS messages can be sent/received.
// @uptrace{SWS_DM_00337}
using ChannelID = uint32_t;

// @brief The identifier of an Uds Transport Protocol implementation.
// @uptrace{SWS_DM_00336}
using UdsTransportProtocolHandlerID = uint8_t;
}  // namespace uds_transport
}  // namespace diag
}  // namespace ara
#endif  // APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_TYPES_H_
