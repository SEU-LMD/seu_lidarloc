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

#ifndef APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_MGR_H
#define APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_MGR_H

#include <cstddef>
#include <tuple>
#include <utility>  // std::pair
#include "public/ara/diag/uds_transport/protocol_types.h"
#include "public/ara/diag/uds_transport/uds_message.h"

namespace ara {
namespace diag {
namespace uds_transport {
// @uptrace{SWS_DM_00306}
class UdsTransportProtocolMgr {
public:
    // @uptrace{SWS_DM_00384}
    enum IndicationResult {
        INDICATION_OK = 0,
        INDICATION_BUSY = 1,
        INDICATION_OVERFLOW = 2,
        INDICATION_UNKNOWN_TARGET_ADDRESS = 3
    };
public:
    UdsTransportProtocolMgr() = default;
    virtual ~UdsTransportProtocolMgr() = default;

    /// @brief Notification call from the given transport channel, that it has been reestablished.
    /// @param GlobalChannelIdentifier global_channel_id,
    /// GlobalChannelIdentifier is type of std::tuple<UdsTransportProtocolHandlerID, ChannelID>
    /// @uptrace{SWS_DM_00313}
    virtual void ChannelReestablished() const = 0;

    /// @brief Hands over a valid received Uds message from transport layer to session layer.
    /// @param UdsMessagePtr message
    /// @uptrace{SWS_DM_00311}
    virtual void HandleMessage() const = 0;

    /// @brief Notification from handler, that it has stopped now.
    /// @param UdsTransportProtocolHandlerID handlerId
    /// @uptrace{SWS_DM_00314}
    virtual void HandlerStopped() const = 0;

    /// @brief Indicates a message start.
    /// @param UdsMessage::Address source_addr,UdsMessage::Address target_addr,UdsMessage::TargetAddressType type,
    /// @param GlobalChannelIdentifier global_channel_id,std::size_t size,Priority indicatePriority,
    /// @param ProtocolKind indicateProtocolKind
    /// @uptrace{SWS_DM_00309}
    virtual std::pair<IndicationResult, UdsMessagePtr> IndicateMessage() const = 0;

    /// @brief Indicates, that the message has failure.
    /// @param UdsMessagePtr message
    /// @uptrace{SWS_DM_00310}
    virtual void NotifyMessageFailure() const = 0;

    /// @brief Notification about the outcome of a transmit request.
    /// @param UdsMessageConstPtr message, TransmissionResult result
    /// @uptrace{SWS_DM_00312}
    virtual void TransmitConfirmation() const = 0;
private:
    UdsTransportProtocolMgr& operator=(const UdsTransportProtocolMgr& other) = default;
    UdsTransportProtocolMgr(const UdsTransportProtocolMgr& other) = default;
};
} /* namespace uds_transport */
} /* namespace diag */
} /* namespace ara */

#endif  // APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_MGR_H_
