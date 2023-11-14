/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  MappingBase.h Mapping框架基类
 */
#ifndef HAF_ADSF_MAPPING_BASE_H
#define HAF_ADSF_MAPPING_BASE_H
#include <shared_mutex>
#include "adsfi_base.h"
#include "location/location.h"

#include "adsf/camera_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/location_send_base.h"
namespace Adsfi {
    class MappingBase : public AdsfiBase {
    public:
        explicit MappingBase(const std::string configFile)
            :   AdsfiBase(configFile),
                subThreads() {};
        ~MappingBase() override;
        void Stop() override;
        std::vector<std::shared_ptr<HafLocation>> GetLocation(const size_t &length) const;
        std::vector<std::shared_ptr<ImageFrameV2>> GetImage(const size_t &length) const;
        HafStatus SendMapping(std::shared_ptr<HafLocation> &data);
        uint32_t GetLocationInstanceIdx() const;
        uint32_t GetImageInstanceIdx() const;
        uint32_t GetMappingInstanceIdx() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInstanceIdxValid() const;
        uint32_t locationInstanceIdx_{0U};
        uint32_t imageInputInstanceIdx_{0U};
        uint32_t mappingInstanceIdx_{0U};
        const uint32_t inputMaskInstanceIdxMin_ = 1401U;
        const uint32_t inputMaskInstanceIdxMax_ = 1415U;
        const uint32_t defaultLocationIns_ = 113U;
        const uint32_t defaultMappingIns_ = 114U;
        std::unique_ptr<LocationReceiveBase> locationReportPtr_{nullptr};
        std::unique_ptr<CameraReceiveBase> imageReportPtr_{nullptr};
        std::unique_ptr<LocationSendBase> mappingSendPtr_{nullptr};
        std::vector<std::thread> subThreads;
    };
}
#endif  // HAF_ADSF_FUSION_LOCATION_H