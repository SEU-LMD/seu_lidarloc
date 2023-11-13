/*
 * FUNCTION: Define RadarTrackArrayReceiveBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_ADSF_RADAR_TRACK_ARRAY_RECEIVE_BASE_H
#define HAF_ADSF_RADAR_TRACK_ARRAY_RECEIVE_BASE_H

#include <shared_mutex>
#include "ara/rtrack/radartrackserviceinterface_proxy.h"
#include "adsf/data_receive_base.h"
#include "core/types.h"
namespace Adsfi {
    class RadarTrackArrayReceiveBase : public DataReceiveBase<ara::rtrack::proxy::RadarTrackServiceInterfaceProxy,
        ara::rtrack::proxy::RadarTrackServiceInterfaceProxy::HandleType, HafRadarTrackArrayFrame> {
    public:
        explicit RadarTrackArrayReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx){};
        virtual ~RadarTrackArrayReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    private:
        static void RadarTrackDataAssign(const ara::rtrack::RadarTrackArray &radarTrackSample,
            HafRadarTrackArrayFrame &data);
        static void SetParams(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetPosition(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetVelocity(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetAcceleration(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetSize(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetOrient(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
        static void SetYaw(HafRadarTrackData &radarTrackTmp, const ara::rtrack::RadarTrack it);
    };
}
#endif // RADAR_TRACK_ARRAY_RECEIVE_BASE_H
