/**
 * FUNCTION: Define Road Feature Detection Base Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 **/
#ifndef HAF_ADSF_ROAD_FEATURE_BASE_H
#define HAF_ADSF_ROAD_FEATURE_BASE_H
#include <shared_mutex>
#include "adsf/adsfi_base.h"
#include "core/core.h"
#include "object/object.h"
#include "adsf/camera_receive_base.h"
#include "adsf/image_send_base.h"

namespace Adsfi {
    class RoadFeatureBase : public AdsfiBase {
    public:
        explicit RoadFeatureBase(const std::string configFile)
            :   AdsfiBase(configFile),
                inputCameraInstanceIdx_(0U),
                outputMaskInstanceIdx_(0U),
                cameraRecv_(nullptr),
                imageSend_(nullptr){};
        ~RoadFeatureBase() override;
        void Stop() override;
        uint32_t GetRecvImgInsIdx() const;
        uint32_t GetOutMaskInsIdx() const;
        HafStatus GetImage(std::shared_ptr<ImageFrameV2> &data, const uint32_t timeout = UINT32_MAX);
        HafStatus SendResult(std::shared_ptr<ImageFrameV2> &data);

    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;

    private:
        bool IsInstanceIdxValid() const;

        uint32_t inputCameraInstanceIdx_;
        uint32_t outputMaskInstanceIdx_;
        const uint32_t inputCameraInstanceIdxMin_ = 21U;
        const uint32_t inputCameraInstanceIdxMax_ = 35U;
        const uint32_t outputMaskInstanceIdxMin_ = 1401U;
        const uint32_t outputMaskInstanceIdxMax_ = 1415U;
        std::unique_ptr<CameraReceiveBase> cameraRecv_;
        std::unique_ptr<ImageSendBase> imageSend_;
        std::vector<std::thread> threadPool_;
    };
}
#endif
