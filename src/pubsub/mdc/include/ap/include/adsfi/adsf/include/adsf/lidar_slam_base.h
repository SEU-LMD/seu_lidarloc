/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  LidarSlamBase.h
 */
#ifndef HAF_ADSF_LIDAR_SLAM_BASE_H
#define HAF_ADSF_LIDAR_SLAM_BASE_H
#include <shared_mutex>
#include "adsfi_base.h"
#include "location/location.h"

#include "adsf/camera_receive_base.h"
#include "adsf/lidar_receive_base.h"
#include "adsf/imu_receive_base.h"
#include "adsf/gnss_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/location_send_base.h"
namespace Adsfi {
    class LidarSlamBase : public AdsfiBase {
    public:
        explicit LidarSlamBase(const std::string configFile) : AdsfiBase(configFile),
            subThreads_() {};
        ~LidarSlamBase() override;
        void Stop() override;

        std::vector<std::shared_ptr<HafLocation>> GetLocation(const size_t &length) const;
        std::vector<std::shared_ptr<ImageFrameV2>> GetImage(const size_t &length) const;
        std::vector<std::shared_ptr<HafIMU>> GetImu(const size_t &length) const;
        std::vector<std::shared_ptr<HafGnssInfo>> GetGnss(const size_t &length) const;
        std::vector<std::shared_ptr<LidarFrame<PointXYZIRT>>> GetLidar(const size_t &length) const;
        HafStatus SendLidarSlam(std::shared_ptr<HafLocation> &data);
        uint32_t GetImageInstanceIdx() const;
        uint32_t GetImuInstanceIdx() const;
        uint32_t GetGnssInstanceIdx() const;
        uint32_t GetLidarInstanceIdx() const;
        uint32_t GetLocationRInstanceIdx() const;
        uint32_t GetLocationPInstanceIdx() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
#ifdef MDC_PRODUCTION_MDC300
        const uint32_t defaultImuInstanceIdx = 2U;
        const uint32_t defaultGnssInstanceIdx = 2U;
#else
        const uint32_t defaultImuInstanceIdx = 1U;
        const uint32_t defaultGnssInstanceIdx = 1U;
#endif
        const uint32_t defaultLocationRInstanceIdx = 113U;
        const uint32_t defaultLocationPInstanceIdx = 115U;
        const uint32_t defaultLidarInstanceMin = 3U;
        const uint32_t defaultLidarInstanceMax = 7U;
        const uint32_t defaultCameraInstanceIdxMin = 21U;
        const uint32_t defaultCameraInstanceIdxMax = 35U;

        bool IsInstanceIdxValid() const;
        uint32_t locationRInstanceIdx_ = defaultLocationRInstanceIdx;
        uint32_t locationPInstanceIdx_ = defaultLocationPInstanceIdx;
        uint32_t imuInstanceIdx_ = defaultImuInstanceIdx;
        uint32_t gnssInstanceIdx_ = defaultGnssInstanceIdx;
        uint32_t lidarInstanceIdx_ = 0U;
        uint32_t imageInstanceIdx_ = 0U;

        std::unique_ptr<LocationReceiveBase> locationReportPtr_{nullptr};
        std::unique_ptr<CameraReceiveBase> imageReportPtr_{nullptr};
        std::unique_ptr<LidarReceiveBase> lidarReportPtr_{nullptr};
        std::unique_ptr<ImuReceiveBase> imuReportPtr_{nullptr};
        std::unique_ptr<GnssReceiveBase> gnssReportPtr_{nullptr};
        std::unique_ptr<LocationSendBase> lidarSlamSendPtr_{nullptr};
        std::vector<std::thread> subThreads_;
    };
}
#endif