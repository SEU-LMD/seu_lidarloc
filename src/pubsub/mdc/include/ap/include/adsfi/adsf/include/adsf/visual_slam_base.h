/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  VisualSlamBase.h
 */
#ifndef HAF_ADSF_VISUAL_SLAM_BASE_H
#define HAF_ADSF_VISUAL_SLAM_BASE_H
#include <shared_mutex>
#include "adsfi_base.h"
#include "location/location.h"

#include "adsf/camera_receive_base.h"
#include "adsf/fusion_object_receive_base.h"
#include "adsf/imu_receive_base.h"
#include "adsf/gnss_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/location_send_base.h"
namespace Adsfi {
    class VisualSlamBase : public AdsfiBase {
    public:
        explicit VisualSlamBase(const std::string configFile) : AdsfiBase(configFile),
            subThreads_() {};
        ~VisualSlamBase() override;
        void Stop() override;

        std::vector<std::shared_ptr<HafLocation>> GetLocation(const size_t &length) const;
        std::vector<std::shared_ptr<ImageFrameV2>> GetImage(const size_t &length) const;
        std::vector<std::shared_ptr<HafIMU>> GetImu(const size_t &length) const;
        std::vector<std::shared_ptr<HafGnssInfo>> GetGnss(const size_t &length) const;
        std::vector<std::shared_ptr<HafFusionOutArray<float32_t>>> GetFusion(const size_t& length) const;
        HafStatus SendVisualSlam(std::shared_ptr<HafLocation> &data);
        uint32_t GetImageInstanceIdx() const;
        uint32_t GetImuInstanceIdx() const;
        uint32_t GetGnssInstanceIdx() const;
        uint32_t GetFusionInstanceIdx() const;
        uint32_t GetLocationRInstanceIdx() const;
        uint32_t GetLocationPInstanceIdx() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
#ifdef MDC_PRODUCTION_MDC300
        const uint32_t DEFAULT_IMU_INSTANCE_IDX = 2U;
        const uint32_t DEFAULT_GNSS_INSTANCE_IDX = 2U;
#else
        const uint32_t DEFAULT_IMU_INSTANCE_IDX = 1U;
        const uint32_t DEFAULT_GNSS_INSTANCE_IDX = 1U;
#endif
        const uint32_t DEFAULT_LOCATION_R_INSTANCE_IDX = 113U;
        const uint32_t DEFAULT_LOCATION_P_INSTANCE_IDX = 116U;
        const uint32_t DEFAULT_FUSION_OBJ_ARRAY_INSTANCE_IDX = 4001U;
        const uint32_t DEFAULT_CAMERA_INSTANCE_IDX_MIN = 21U;
        const uint32_t DEFAULT_CAMERA_INSTANCE_IDX_MAX = 35U;

        bool IsInstanceIdxValid() const;
        uint32_t locationRInstanceIdx_ = DEFAULT_LOCATION_R_INSTANCE_IDX;
        uint32_t locationPInstanceIdx_ = DEFAULT_LOCATION_P_INSTANCE_IDX;
        uint32_t imuInstanceIdx_ = DEFAULT_IMU_INSTANCE_IDX;
        uint32_t gnssInstanceIdx_ = DEFAULT_GNSS_INSTANCE_IDX;
        uint32_t fusionObjArrayInstanceIdx_ = DEFAULT_FUSION_OBJ_ARRAY_INSTANCE_IDX;
        uint32_t imageInstanceIdx_ = 0U;

        std::unique_ptr<LocationReceiveBase> locationReportPtr_{nullptr};
        std::unique_ptr<CameraReceiveBase> imageReportPtr_{nullptr};
        std::unique_ptr<ImuReceiveBase> imuReportPtr_{nullptr};
        std::unique_ptr<GnssReceiveBase> gnssReportPtr_{nullptr};
        std::unique_ptr<LocationSendBase> visualSlamSendPtr_{nullptr};
        std::unique_ptr<FusionObjectReceiveBase> fusionObjectArrayReportPtr_{nullptr};
        std::vector<std::thread> subThreads_;
    };
}
#endif