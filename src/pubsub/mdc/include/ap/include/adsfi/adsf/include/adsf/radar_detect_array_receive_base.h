/*
 * FUNCTION: Define RadarDetectArrayReceiveBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_ADSF_RADAR_DETECT_ARRAY_RECEIVE_BASE_H
#define HAF_ADSF_RADAR_DETECT_ARRAY_RECEIVE_BASE_H

#include <shared_mutex>
#include "ara/rdetect/radardetectserviceinterface_proxy.h"
#include "adsf/data_receive_base.h"
#include "core/types.h"
namespace Adsfi {
    class RadarDetectArrayReceiveBase
        : public DataReceiveBase<ara::rdetect::proxy::RadarDetectServiceInterfaceProxy,
        ara::rdetect::proxy::RadarDetectServiceInterfaceProxy::HandleType, HafRadarDetectArrayFrame> {
    public:
        explicit RadarDetectArrayReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx){};
        virtual ~RadarDetectArrayReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    private:
        static void RadarDetectDataAssign(const ara::rdetect::RadarDetectArray &radarDetectSample,
            HafRadarDetectArrayFrame &data);
        static void SetParams(HafRadarDetectData &radarDetectTmp, const ara::rdetect::RadarDetect it);
        static void SetPosition(HafRadarDetectData &radarDetectTmp, const ara::rdetect::RadarDetect it);
        static void SetVelocity(HafRadarDetectData &radarDetectTmp, const ara::rdetect::RadarDetect it);
    };
}
#endif  // RADAR_DETECT_ARRAY_RECEIVE_BASE_H
