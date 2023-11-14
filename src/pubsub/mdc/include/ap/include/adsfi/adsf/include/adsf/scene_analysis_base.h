 /*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  SceneAnalysisBase.h 场景管理框架基类
 * Create: 2020-08-07
 */
#ifndef ADSF_SCENE_ANALYSIS_BASE
#define ADSF_SCENE_ANALYSIS_BASE
#include <shared_mutex>
#include "core/core.h"
#include "dnn/dnn.h"
#include "adsfi_base.h"
#include "location/location.h"
#include "location/gnss_info.h"
#include "location/imu.h"
#include "adsf/ego_trajectory_receive_base.h"
#include "adsf/fusion_object_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/navigation_result_receive_base.h"
#include "adsf/chassis_report_receive_base.h"
#include "adsf/camera_receive_base.h"
#include "adsf/ins_receive_base.h"
#include "adsf/scene_feature_send_base.h"
#include "adsf/fm_receive_base.h"
#include "adsf/prediction_receive_base.h"

namespace Adsfi {
    class SceneAnalysisBase : public AdsfiBase {
    public:
        explicit SceneAnalysisBase(const std::string configFile)
            : AdsfiBase(configFile),
            locationInstanceIdx_(113U),
            objectArrayInstanceIdx_(4001U),
            navigationResultInstanceIdx_(110U),
            egoTrajectoryInstanceIdx_(103U),
            chassisReportInstanceIdx_(1U),
            imageInputInstanceIdx_(21U),
            insInputInstanceIdx_(1U),
            fmInstanceIdx_(1444U),
            predictionInstanceIdx_(4681U),
            sammFeatureInstanceIdx_(6663U),
            fmRecTimeOutMs_(10) {};
        ~SceneAnalysisBase() override;
        void Stop() override;
        std::vector<std::shared_ptr<HafLocation>> GetLocation(const size_t &length) const;
        std::vector<std::shared_ptr<HafFusionOutArray<float32_t>>> GetObjectArray(const size_t &length) const;
        std::vector<std::shared_ptr<HafNavigationResult>> GetNavigationResult(const size_t &length) const;
        std::vector<std::shared_ptr<HafEgoTrajectory>> GetEgoTrajectory(const size_t &length) const;
        std::vector<std::shared_ptr<HafChassisReport>> GetChassisReport(const size_t &length) const;
        std::vector<std::shared_ptr<ImageFrameV2>> GetCamera(const size_t &length) const;
        std::vector<std::shared_ptr<HafInsInfo>> GetIns(const size_t &length) const;
        HafStatus GetFm(std::vector<HafMdcFaultEvent>& fmData) const;
        std::vector<std::shared_ptr<HafObjPredictionOutArray>> GetPrediction(const size_t &length) const;
        HafStatus SendResult(std::shared_ptr<HafSammFeature> &data);
        uint32_t GetLocationIdx() const;
        uint32_t GetObjectArrayIdx() const;
        uint32_t GetNavigationResultIdx() const;
        uint32_t GetEgoTrajectoryIdx() const;
        uint32_t GetChassisReportIdx() const;
        uint32_t GetCameraIdx() const;
        uint32_t GetInsIdx() const;
        uint32_t GetFmIdx() const;
        uint32_t GetPredictionIdx() const;
        uint32_t GetResultIdx() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInstanceIdxValid() const;
        bool IsCameraInstanceIdxValid() const;
        std::vector<std::thread> threadPool_;
        const uint32_t defaultLocationInstanceIdx_{113U};
        const uint32_t defaultObjectArrayInstanceIdx_{4001U};
        const uint32_t defaultNavigationResultInstanceIdx_{110U};
        const uint32_t defaultEgoTrajectoryInstanceIdx_{103U};
        const uint32_t defaultChassisReportInstanceIdx_{1U};
        const uint32_t defaultImageInputInstanceIdxMin_{21U};
        const uint32_t defaultImageInputInstanceIdxMax_{35U};
#ifdef MDC_PRODUCTION_MDC300
        const uint32_t defaultInsInputInstanceIdx_{2U};
#else
        const uint32_t defaultInsInputInstanceIdx_{1U};
#endif
        const uint32_t defaultSammFeatureInstanceIdx_{6663U};
        const uint32_t defaultFmInstanceIdx_{1444U};
        const uint32_t defaultPredictionInstanceIdx_{4681U};
        uint32_t locationInstanceIdx_;
        uint32_t objectArrayInstanceIdx_;
        uint32_t navigationResultInstanceIdx_;
        uint32_t egoTrajectoryInstanceIdx_;
        uint32_t chassisReportInstanceIdx_;
        uint32_t imageInputInstanceIdx_;
        uint32_t insInputInstanceIdx_;
        uint32_t fmInstanceIdx_;
        uint32_t predictionInstanceIdx_;
        uint32_t sammFeatureInstanceIdx_;
        int32_t fmRecTimeOutMs_;
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::unique_ptr<FusionObjectReceiveBase> objectArrayRecv_{nullptr};
        std::unique_ptr<NavigationResultReceiveBase> navigationResultRecv_{nullptr};
        std::unique_ptr<EgoTrajectoryReceiveBase> egoTrajectoryRecv_{nullptr};
        std::unique_ptr<ChassisReportReceiveBase> chassisReportRecv_{nullptr};
        std::unique_ptr<CameraReceiveBase> cameraRecv_{nullptr};
        std::unique_ptr<InsReceiveBase> insRecv_{nullptr};
        std::unique_ptr<FmReceiveBase> fmRecv_{nullptr};
        std::unique_ptr<PredictionReceiveBase> predictionRecv_{nullptr};
        std::unique_ptr<SceneFeatureSendBase> resultSend_{nullptr};
    };
}
#endif // ADSF_SCENE_ANALYSIS_BASE