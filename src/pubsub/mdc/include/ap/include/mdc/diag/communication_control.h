/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: communication_control.h is designed for communicate control
 */
#ifndef MDC_COMMUNICATION_CONTROL_H
#define MDC_COMMUNICATION_CONTROL_H
#include <vector>
#include <string>
#include <mutex>

#include "mdc/common/result.h"
#include "mdc/common/future.h"
#include "mdc/diag/meta_info.h"
#include "mdc/common/promise.h"
#include "mdc/diag/cancellation_handler.h"
#include "mdc/diag/diag_error_domain/diag_error_domain.h"
#include "mdc/diag/diag_error_domain/diag_uds_nrc_error_domain.h"

namespace mdc {
namespace diag {
class CommunicationControlAgent;
class CommunicationControl {
public:
    /**
    * \brief Common control type definitions. Just for better understanding.
    **/
    static constexpr uint8_t CONTROL_TYPE_ENABLE_RX_TX = 0X00U;
    static constexpr uint8_t CONTROL_TYPE_ENABLE_RX_DISABLE_TX = 0X01U;
    static constexpr uint8_t CONTROL_TYPE_DISABLE_RX_ENABLE_TX = 0X02U;
    static constexpr uint8_t CONTROL_TYPE_DISABLE_RX_TX = 0X03U;
    static constexpr uint8_t CONTROL_TYPE_ENABLE_RX_DISABLE_TX_WITH_ADDRESS = 0X04U;
    static constexpr uint8_t CONTROL_TYPE_ENABLE_RX_AND_TX_WITH_ADDRESS = 0X05U;

    /**
    * \brief Common communication type definitions. Just for better understanding.
    **/
    // This value references all application-related communication
    static constexpr uint8_t COMMUNICATION_TYPE_NORMAL = 0X01U;
    // This value references all network management related communication.
    static constexpr uint8_t COMMUNICATION_TYPE_NETWORK_MANAGEMENT = 0X02U;
    // This value references all network management and application-related communication.
    static constexpr uint8_t COMMUNICATION_TYPE_NORMAL_AND_NETWORK_MANAGEMENT = 0X03U;

    /**
    * \brief ComCtrlRequestParamsType is a structure, which holds all parameters of an UDS 0x28 communicationControl
    * request.
    **/
    struct ComCtrlRequestParamsType {
        uint8_t controlType;
        uint8_t communicationType;
        uint8_t subnetNumber;
        /**
        * This 2 byte parameter is used to identify a node on a sub-network somewhere in the vehicle, which can not be
        * addressed using the addressing methods of the lower OSI layers 1 to 6. This parameter is only present, if the
        * subfunction parameter controlType is set to 0x04 or 0x05
        **/
        uint16_t nodeIdentificationNumber;
    };

    explicit CommunicationControl(const std::string& specifier);
    virtual ~CommunicationControl();
    /**
    * \brief This Offer will enable the DM to forward request messages to this handler. error result type is
    * mdc::diag::DiagOfferErr.
    **/
    mdc::common::Result<void> Offer();

    /**
    * \brief This StopOffer will disable the forwaring of request messages from DM. error result type is
    * mdc::diag::DiagOfferErr.
    **/
    void StopOffer();

    /**
    * \brief Called for CommunicationControl (0x28) with any subfunction as subfunction value is part of argument list.
    * Typically provider of this interface is considered as part of the state management. error result type is
    * mdc::diag::DiagUdsNrcErrc.
    */
    virtual mdc::common::Future<void> CommCtrlRequest(ComCtrlRequestParamsType controlType,
        mdc::diag::MetaInfo metaInfo,  mdc::diag::CancellationHandler cancellationHandler) = 0;
protected:
    CommunicationControl(const CommunicationControl&) = delete;
    CommunicationControl& operator=(const CommunicationControl&) = delete;
    CommunicationControl(CommunicationControl&&);
    CommunicationControl& operator=(CommunicationControl&&);
private:
    std::string mdcComCtrlSpecifier_;
    bool hasOffered_;
    std::recursive_mutex offerMutex_;
    std::shared_ptr<CommunicationControlAgent> agent_;
};
}
}
#endif