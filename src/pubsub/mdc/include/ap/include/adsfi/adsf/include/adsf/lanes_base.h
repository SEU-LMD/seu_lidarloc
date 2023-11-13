/**
 * FUNCTION: Define Lanes Base Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 **/
#ifndef HAF_ADSF_LANES_BASE_H
#define HAF_ADSF_LANES_BASE_H

#include <string>
#include <shared_mutex>
#include "core/core.h"
#include "object/lanes.h"
#include "adsfi_base.h"
#include "camera_receive_base.h"
#include "lanes_send_base.h"

namespace Adsfi {
    class LanesBase : public AdsfiBase {
    public:
        explicit LanesBase(const std::string configFile) : AdsfiBase(configFile) {};
        ~LanesBase() override;
        void Stop() override;
        HafStatus GetImage(std::shared_ptr<ImageFrameV2> &data);
        HafStatus SendLanes(std::shared_ptr<HafLaneDetectionOutArray> &data);
        uint32_t GetCameraInsIdx() const;
        uint32_t GetResultLanesInsIdx() const;

    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode &config) override;
    private:
        bool IsInstanceIdxValid() const;

        const uint32_t defaultcameraInstanceIdx_ { 21U };
        const uint32_t defaultlanesOutputInstanceIdx_ { 1201U };

        const uint32_t cameraInstanceIdxMin_ { 21U };
        const uint32_t cameraInstanceIdxMax_ { 35U };
        const uint32_t lanesOutputIntanceIdxMin_ { 1201U };
        const uint32_t lanesOutputIntanceIdxMax_ { 1215U };

        uint32_t cameraInstanceIdx_ {defaultcameraInstanceIdx_};
        uint32_t lanesOutputInstanceIdx_ {defaultlanesOutputInstanceIdx_};

        std::unique_ptr<CameraReceiveBase> cameraRecv_ { nullptr };
        std::unique_ptr<LanesSendBase> lanesSend_ { nullptr };
        std::vector<std::thread> threadPool_;
    };
}
#endif  // HAF_ADSF_LANES_BASE_H__