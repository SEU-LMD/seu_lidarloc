/* *
 * FUNCTION: Define Object3DSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *   */
#ifndef ADSF_TRAFFICLIGHTSENDBASE_H
#define ADSF_TRAFFICLIGHTSENDBASE_H

#include "data_send_base.h"
#include "core/types.h"
#include "object/object.h"

#include "mdc/trafficlight/trafficlightinterface_skeleton.h"

#include <shared_mutex>

namespace Adsfi {
class TrafficLightSendBase
    : public DataSendBase<mdc::trafficlight::skeleton::TrafficLightInterfaceSkeleton,
    HafTlDetectionOutArray<float64_t>> {
public:
    explicit TrafficLightSendBase(const uint32_t idx) : DataSendBase(idx){};
    ~TrafficLightSendBase() override;
    void SendingData() override;
private:
    void TlDataAssign(ara::trafficlight::TrafficLightArray &tlArray,
        const std::shared_ptr<HafTlDetectionOutArray<float64_t>>& data) const;
}; // class TrafficLightSendBase
} // namespace adsfi
#endif
