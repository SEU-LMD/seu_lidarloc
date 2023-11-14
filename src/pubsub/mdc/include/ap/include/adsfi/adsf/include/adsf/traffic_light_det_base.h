/*
 * FUNCTION: Define TrafficLightBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_ADSF_TRAFFIC_LIGHT_DET_BASE_H
#define HAF_ADSF_TRAFFIC_LIGHT_DET_BASE_H

#include <string>
#include <shared_mutex>
#include "core/core.h"
#include "object/object.h"
#include "adsfi_base.h"
#include "camera_receive_base.h"
#include "location_receive_base.h"
#include "traffic_light_send_base.h"

namespace Adsfi {
    class TrafficLightDetBase : public AdsfiBase {
    public:
        explicit TrafficLightDetBase(const std::string configFile)
            :   AdsfiBase(configFile),
                cameraRecv_(nullptr),
                locationRecv_(nullptr),
                resultSend_(nullptr) {}
        ~TrafficLightDetBase() override;
        void Stop() override;
        HafStatus GetImage(std::shared_ptr<ImageFrameV2> &data);
        HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
        HafStatus SendObject(std::shared_ptr<HafTlDetectionOutArray<float64_t>> &data);
        uint32_t GetCameraInsIdx() const;
        uint32_t GetLocInsIdx() const;
        uint32_t GetResultObjInsIdx() const;

        uint32_t detOutInsIdxMin_ = 1031U;
        uint32_t detOutInsIdxMax_ = 1045U;
        uint32_t camInsIdxMin_ = 21U;
        uint32_t camInsIdxMax_ = 35U;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInsIdxValid() const;
        const uint32_t defalutCamInstanceIdx_ = 21U;
        const uint32_t defalutLocationInstanceIdx_ = 113U;
        const uint32_t defaultDetectionOutInstanceIdx_ = 1031U;

        uint32_t cameraInstanceIdx_ = defalutCamInstanceIdx_;
        uint32_t locationInstanceIdx_ = defalutLocationInstanceIdx_;
        uint32_t detectionOutInstanceIdx_ = defaultDetectionOutInstanceIdx_;
        std::unique_ptr<CameraReceiveBase> cameraRecv_{nullptr};
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::unique_ptr<TrafficLightSendBase> resultSend_{nullptr};
        std::vector<std::thread> threadPool_;
    };
}

#endif