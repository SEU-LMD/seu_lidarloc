/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  FusionLocationBase.h 融合定位框架基类
 */

#ifndef HAF_ADSF_FUSION_LOCATION_H
#define HAF_ADSF_FUSION_LOCATION_H

#include <shared_mutex>
#include "adsfi_base.h"
#include "location/location.h"
#include "location/gnss_info.h"
#include "location/imu.h"
#include "adsf/imu_receive_base.h"
#include "adsf/gnss_receive_base.h"
#include "adsf/ins_receive_base.h"
#include "adsf/chassis_report_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/location_send_base.h"
namespace Adsfi {
    class FusionLocationBase : public AdsfiBase {
    public:
        explicit FusionLocationBase(const std::string configFile)
            :   AdsfiBase(configFile) {};
        ~FusionLocationBase() override;
        void Stop() override;
        std::vector<std::shared_ptr<HafIMU>> GetImu(const size_t &length) const;
        std::vector<std::shared_ptr<HafGnssInfo>> GetGnss(const size_t &length) const;
        std::vector<std::shared_ptr<HafInsInfo>> GetIns(const size_t &length) const;
        std::vector<std::shared_ptr<HafChassisReport>> GetChassis(const size_t &length) const;
        std::vector<std::shared_ptr<HafLocation>> GetMapping(const size_t &length) const;
        std::vector<std::shared_ptr<HafLocation>> GetLidarSlam(const size_t &length) const;
        std::vector<std::shared_ptr<HafLocation>> GetVisualSlam(const size_t &length) const;
        std::vector<std::shared_ptr<HafLocation>> GetOdometry(const size_t &length) const;
        HafStatus SendLocation(std::shared_ptr<HafLocation> &data);

    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInsIdxValid() const;
#ifdef MDC_PRODUCTION_MDC300
        const uint32_t defaultImuInputIns_ = 2U;
        const uint32_t defaultGnssInputIns_ = 2U;
        const uint32_t defaultInsInputIns_ = 2U;
#else
        const uint32_t defaultImuInputIns_ = 1U;
        const uint32_t defaultGnssInputIns_ = 1U;
        const uint32_t defaultInsInputIns_ = 1U;
#endif
        const uint32_t defaultChassisInputIns_ = 1U;
        const uint32_t defaultMappingInputIns_ = 114U;
        const uint32_t defaultLidarSlamInputIns_ = 115U;
        const uint32_t defaultVisualSlamInputIns_ = 116U;
        const uint32_t defaultOdometryInputIns_ = 117U;
        const uint32_t defaultLocationResultIns_ = 113U;

        uint32_t imuInputInstanceIdx_{0U};
        uint32_t gnssInputInstanceIdx_{0U};
        uint32_t insInputInstanceIdx_{0U};
        uint32_t chassisReportInputInstanceIdx_{0U};
        uint32_t mappingInputInstanceIdx_{0U};
        uint32_t lidarSlamInputInstanceIdx_{0U};
        uint32_t visualSlamInputInstanceIdx_{0U};
        uint32_t odometryInputInstanceIdx_{0U};
        uint32_t locationResultInstanceIdx_{0U};

        std::unique_ptr<ImuReceiveBase> imuReportPtr_{nullptr};
        std::unique_ptr<GnssReceiveBase> gnssReportPtr_{nullptr};
        std::unique_ptr<InsReceiveBase> insReportPtr_{nullptr};
        std::unique_ptr<ChassisReportReceiveBase> chassisReportPtr_{nullptr};
        std::unique_ptr<LocationReceiveBase> mappingReportPtr_{nullptr};
        std::unique_ptr<LocationReceiveBase> lidarSlamReportPtr_{nullptr};
        std::unique_ptr<LocationReceiveBase> visualSlamReportPtr_{nullptr};
        std::unique_ptr<LocationReceiveBase> odometryReportPtr_{nullptr};
        std::unique_ptr<LocationSendBase> locationSendPtr_{nullptr};

        std::vector<std::thread> threadPool_;
    };
}
#endif // HAF_ADSF_FUSION_LOCATION_H
