/*
 * FUNCTION: Define RadarDetectArrayDetBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_ADSF_RADAR_DETECT_ARRAY_DETBASE_H
#define HAF_ADSF_RADAR_DETECT_ARRAY_DETBASE_H

#include <string>
#include <shared_mutex>
#include "core/core.h"
#include "object/object.h"
#include "adsfi_base.h"
#include "radar_detect_array_receive_base.h"
#include "location_receive_base.h"
#include "object_3d_send_base.h"

namespace Adsfi {
    class RadarDetectArrayDetBase : public AdsfiBase {
    public:
        explicit RadarDetectArrayDetBase(const std::string configFile)
            :   AdsfiBase(configFile),
                radarDetectArrayInstanceIdx_(defalutRadarDetectArrayInstanceIdx_),
                locationInstanceIdx_(defalutLocationInstanceIdx_),
                detectionOutInstanceIdx_(defaultDetectionOutInstanceIdx_),
                radarDetectRecv_(nullptr),
                locationRecv_(nullptr),
                resultSend_(nullptr) {};
        ~RadarDetectArrayDetBase() override;
        void Stop() override;
        HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
        HafStatus GetRadar(std::shared_ptr<HafRadarDetectArrayFrame> &data);
        HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);
        uint32_t GetRadarInsIdx() const;
        uint32_t GetResultObjInsIdx() const;
        uint32_t GetLocInsIdx() const;

    protected:
        HafStatus StartPubThread() override;
        HafStatus StartSubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInsIdxValid() const;
#if defined(MDC_PRODUCTION_CORE)
        const uint32_t defalutRadarDetectArrayInstanceIdx_ = 1U;
#else
        const uint32_t defalutRadarDetectArrayInstanceIdx_ = 3U;
#endif
        const uint32_t defalutLocationInstanceIdx_ = 113U;
        const uint32_t defaultDetectionOutInstanceIdx_ = 3001U;
        const uint32_t detOutInsIdxMin_ = 3001U;
        const uint32_t detOutInsIdxMax_ = 3006U;
        uint32_t radarDetectArrayInstanceIdx_;
        uint32_t locationInstanceIdx_;
        uint32_t detectionOutInstanceIdx_;
        std::unique_ptr<RadarDetectArrayReceiveBase> radarDetectRecv_{nullptr};
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::unique_ptr<Object3DSendBase> resultSend_{nullptr};
        std::vector<std::thread> threadPool_;
    };
}
#endif
