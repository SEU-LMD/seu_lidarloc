/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 */

#ifndef APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_HANDLER_H
#define APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_HANDLER_H

#include "public/ara/diag/uds_transport/protocol_types.h"
#include "ara/diag/uds_transport/uds_transport_protocol_mgr.h"

namespace ara {
namespace diag {
namespace uds_transport {
/// @brief Abstract Class, which a specific UDS Transport Protocol (plugin) shall subclass.
/// @uptrace{SWS_DM_00315}
class UdsTransportProtocolHandler {
public:
    /// @uptrace{SWS_DM_00315}
    enum class InitializationResult {
        INITIALIZE_OK = 0,
        INITIALIZE_FAILED = 1,
    };

public:
    /// @brief Constructor of UdsTransportProtocolHandler.
    /// @uptrace{SWS_DM_09015}
    explicit UdsTransportProtocolHandler(const UdsTransportProtocolHandlerID handlerId,
        UdsTransportProtocolMgr& transportProtocolMgr)
        : transportProtocolManager_(transportProtocolMgr), handlerId_(handlerId)
    {
    }
    /// @brief Destructor of UdsTransportProtocolHandler.
    /// @uptrace{SWS_DM_09016}
    virtual ~UdsTransportProtocolHandler() = default;

    /// @brief Return the UdsTransportProtocolHandlerID, which was given to the implementation
    /// during construction (ctorcall).
    /// @uptrace{SWS_DM_00325}
    virtual UdsTransportProtocolHandlerID GetHandlerId() const
    {
        return handlerId_;
    }

    /// @brief Initializes handler.
    /// @uptrace{SWS_DM_00319}
    virtual InitializationResult Initialize() const = 0;

    /// @brief Tells the UdsTransportProtocolHandler, that it shall notify the DM core via UdsTransport
    /// ProtocolMgr::ChannelReestablished()) if the given channel has been re-established after next Start().
    /// @uptrace{SWS_DM_00326}
    virtual bool NotifyReestablishment() const = 0;

    /// @brief Start processing the implemented Uds Transport Protocol.
    /// @uptrace{SWS_DM_00322}
    virtual bool Start() = 0;

    /// @brief Method to indicate that this UdsTransportProtocolHandler should terminate.
    /// @uptrace{SWS_DM_00323}
    virtual void Stop() = 0;

    /// @brief Transmit a Uds message via the underlying Uds Transport Protocol channel.
    /// @uptrace{SWS_DM_00327}
    virtual void Transmit(UdsMessageConstPtr message, ChannelID transmitChannelId) const = 0;

protected:
    UdsTransportProtocolHandler(const UdsTransportProtocolHandler& other) = default;
    UdsTransportProtocolHandler& operator=(const UdsTransportProtocolHandler& other) = default;
    /// @brief The UdsTransportProtocolMgr used/provided by the DM/DCM.
    /// @uptrace{SWS_DM_00315}
    UdsTransportProtocolMgr& transportProtocolManager_;

private:
    /// @brief The id of the handler (shall be set by initializer list of ctor.)
    /// @uptrace{SWS_DM_00315}
    const ara::diag::uds_transport::UdsTransportProtocolHandlerID handlerId_;
};
} /* namespace uds_transport */
} /* namespace diag */
} /* namespace ara */

#endif  // APD_ARA_DIAG_UDS_TRANSPORT_PROTOCOL_HANDLER_H_
