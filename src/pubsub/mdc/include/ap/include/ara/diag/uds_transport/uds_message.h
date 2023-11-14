/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 */


/// @file

#ifndef APD_ARA_DIAG_UDS_TRANSPORT_UDS_MESSAGE_H
#define APD_ARA_DIAG_UDS_TRANSPORT_UDS_MESSAGE_H

#include <cstdint>  // uint8_t
#include <memory>  // unique_ptr
#include "ara/core/map.h"
#include "ara/core/string.h"
#include "public/ara/diag/uds_transport/protocol_types.h"  // ByteVector

namespace ara {
namespace diag {
namespace uds_transport {
    // Data identifier size in uds message
    const size_t DID_DATA_SIZE = 2U;

    // Routine identifier size in uds message
    const size_t RID_SIZE = 2U;

/// @brief Class which represents a UDS message
/// @uptrace **SWS_DM_00291**
class UdsMessage {
public:
    // @brief Type for UDS source and target addresses.
    // @uptrace{SWS_DM_00293}
    using Address = uint16_t;
    // @brief Type for the meta information attached to a UdsMessage.
    // @uptrace{SWS_DM_00294}
    using MetaInfoMap = ara::core::Map<ara::core::String, ara::core::String>;
    // @brief Type of target address in UdsMessage.
    // @uptrace{SWS_DM_00296}
    enum class TargetAddressType : std::uint8_t {
        kPhysical = 0,
        kFunctional = 1
    };
    using PayloadSizeType = ara::diag::uds_transport::ByteVector::size_type;

public:
    /// @uptrace **SWS_DM_09010**
    virtual ~UdsMessage() = default;

    /// @uptrace{SWS_DM_00302}
    virtual void AddMetaInfo(std::unique_ptr<const MetaInfoMap> metaInfo) const;

    /// @brief Get the UDS message data starting with the SID (A_Data as per ISO)
    /// @uptrace{SWS_DM_00300}
    virtual const uds_transport::ByteVector& GetUdsPayload() const;

    /// @brief Get the source address of the uds message.
    /// @uptrace{SWS_DM_00297}
    virtual Address GetSa() const noexcept;

    /// @brief Get the target address of the uds message.
    /// @uptrace{SWS_DM_00298}
    virtual Address GetTa() const noexcept;

    /// @brief Get the target address type of the uds message.
    /// @uptrace{SWS_DM_00299}
    virtual TargetAddressType GetTaType() const noexcept;

protected:
    // @brief non public default ctor
    // @uptrace{SWS_DM_09012}
    UdsMessage() = default;
    UdsMessage(const Address udsSa,
        const Address udsTa,
        const uds_transport::ByteVector& udsPayload = {});

    UdsMessage(const Address udsSa,
        const Address udsTa,
        const TargetAddressType requestTa,
        const uds_transport::ByteVector& udsPayload = {});

private:
    // @brief Payload of the uds message.
    // @uptrace ***SWS_DM_00291***
    // @uptrace ***SWS_DM_09028***
    ara::diag::uds_transport::ByteVector payload_ {};

private:
    Address sourceAddress_ {0};
    Address targetAddress_ {0};
    TargetAddressType requestTaType_; // request target address type. if this is a response, it means the sa type.
};

/// @brief unique_ptr for constant UdsMessages as provided by the generic/core DM part towards the
/// UdsTransportLayer-Plugin.
/// @uptrace{SWS_DM_00304}
using UdsMessageConstPtr = std::unique_ptr<const UdsMessage>;

/// @brief unique_ptr for UdsMessagesas provided by the generic/core DM part towards the UdsTransportLayer-Plugin.
/// @uptrace{SWS_DM_00303}
using UdsMessagePtr = std::unique_ptr<UdsMessage>;
} /* namespace uds_transport */
} /* namespace diag */
} /* namespace ara */

#endif  // APD_ARA_DIAG_UDS_TRANSPORT_UDS_MESSAGE_H_
