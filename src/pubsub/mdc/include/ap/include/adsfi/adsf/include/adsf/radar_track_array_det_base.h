/*
 * FUNCTION: Define RadarTrackArrayDetBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_ADSF_RADAR_TRACK_ARRAY_DET_BASE_H
#define HAF_ADSF_RADAR_TRACK_ARRAY_DET_BASE_H

#include <string>
#include <shared_mutex>
#include "core/core.h"
#include "object/object.h"
#include "adsfi_base.h"
#include "radar_track_array_receive_base.h"
#include "location_receive_base.h"
#include "object_3d_send_base.h"

namespace Adsfi {
    class RadarTrackArrayDetBase : public AdsfiBase {
    public:
        explicit RadarTrackArrayDetBase(const std::string configFile)
            :   AdsfiBase(configFile),
                radarTrackArrayInstanceIdx_(defalutRadarTrackArrayInstanceIdx_),
                locationInstanceIdx_(defalutLocationInstanceIdx_),
                detectionOutInstanceIdx_(defaultDetectionOutInstanceIdx_),
                radarTrackRecv_(nullptr),
                locationRecv_(nullptr),
                resultSend_(nullptr) {};
        ~RadarTrackArrayDetBase() override;
        void Stop() override;
        HafStatus GetRadar(std::shared_ptr<HafRadarTrackArrayFrame> &data);
        HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
        HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);
        uint32_t GetRadarInsIdx() const;
        uint32_t GetLocInsIdx() const;
        uint32_t GetResultObjInsIdx() const;

    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInsIdxValid() const;
#if defined(MDC_PRODUCTION_CORE)
        const uint32_t defalutRadarTrackArrayInstanceIdx_ = 1U;
#else
        const uint32_t defalutRadarTrackArrayInstanceIdx_ = 4U;
#endif
        const uint32_t defalutLocationInstanceIdx_ = 113U;
        const uint32_t defaultDetectionOutInstanceIdx_ = 3001U;
        const uint32_t detOutInsIdxMin_ = 3001U;
        const uint32_t detOutInsIdxMax_ = 3006U;
        uint32_t radarTrackArrayInstanceIdx_;
        uint32_t locationInstanceIdx_;
        uint32_t detectionOutInstanceIdx_;
        std::unique_ptr<RadarTrackArrayReceiveBase> radarTrackRecv_{nullptr};
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::unique_ptr<Object3DSendBase> resultSend_{nullptr};
        std::vector<std::thread> threadPool_;
    };
}
#endif
