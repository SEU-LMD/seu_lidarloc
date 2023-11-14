/* *
 * FUNCTION: Define ObjectReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *   */
#ifndef ADSF_TRAFFICLIGHTRECEIVEBASE_H
#define ADSF_TRAFFICLIGHTRECEIVEBASE_H

#include "data_receive_base.h"
#include "core/types.h"
#include "object/object.h"

#include <shared_mutex>

#include "mdc/trafficlight/trafficlightinterface_proxy.h"
namespace Adsfi {
    class TrafficLightReceiveBase
        : public DataReceiveBase<mdc::trafficlight::proxy::TrafficLightInterfaceProxy,
        mdc::trafficlight::proxy::TrafficLightInterfaceProxy::HandleType,
        HafTlDetectionOutArray<float64_t>> {
    public:
        explicit TrafficLightReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx) {}
        virtual ~TrafficLightReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    private:
        void TlDataAssign(const ara::trafficlight::TrafficLightArray &objSample,
            HafTlDetectionOutArray<float64_t> &data) const;
    }; // class TrafficLightReceiveBase
} // namespace adsfi
#endif
