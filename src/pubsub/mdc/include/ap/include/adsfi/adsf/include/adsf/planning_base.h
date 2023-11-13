/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  PlanningBase.h Planning规划框架基类
 */

#ifndef HAF_ADSF_PLANNING_BASE_H
#define HAF_ADSF_PLANNING_BASE_H

#include <shared_mutex>
#include "adsfi_base.h"
#include "adsf/ego_trajectory_send_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/fusion_object_receive_base.h"
#include "adsf/object_3d_receive_base.h"
#include "adsf/prediction_receive_base.h"
#include "adsf/lanes_receive_base.h"
#include "adsf/traffic_light_receive_base.h"
#include "adsf/navigation_result_receive_base.h"
#include "adsf/chassis_report_receive_base.h"
#include "object/object.h"
#include "object/lanes.h"
#include "object/prediction.h"

namespace Adsfi {
class PlanningBase : public AdsfiBase {
public:
    explicit PlanningBase(const std::string configFile)
        : AdsfiBase(configFile),
          locationInstanceIdx_(0U),
          navigationResultInstanceIdx_(0U),
          chassisReportInstanceIdx_(0U),
          egoTrajectoryInstanceIdx_(0U),
          fusionObjectArrayInstanceIdx_(0U),
          fusionLaneDetectionOutArrayInstanceIdx_(0U),
          predictionOutArrayInstanceIdx_(0U),
          tlDetectionOutArrayInstanceIdx_(0U){};
    ~PlanningBase() override;
    void Stop() override;
    std::vector<std::shared_ptr<HafLocation>> GetNLocationData(const size_t& num) const;
    std::vector<std::shared_ptr<HafChassisReport>> GetNChassisReportData(const size_t& num) const;
    std::vector<std::shared_ptr<HafNavigationResult>> GetNNavigationResultData(const size_t& num) const;
    std::vector<std::shared_ptr<HafFusionOutArray<float32_t>>> GetNFusionOutObjectData(const size_t& num) const;
    std::vector<std::shared_ptr<HafLaneDetectionOutArray>> GetNFusionLaneData(const size_t& num) const;
    std::vector<std::shared_ptr<HafObjPredictionOutArray>> GetNPredictionTrajectoryData(const size_t& num) const;
    std::vector<std::shared_ptr<HafTlDetectionOutArray<float64_t>>> GetNTrafficLightData(const size_t& num) const;

    HafStatus SendEgoTrajectory(std::shared_ptr<HafEgoTrajectory>& data) const;

    uint32_t GetLocationInsIdx() const;
    uint32_t GetChassisReportInsIdx() const;
    uint32_t GetNavigationResultInsIdx() const;
    uint32_t GetFusionOutObjectsInsIdx() const;
    uint32_t GetFusionLanesInsIdx() const;
    uint32_t GetPredictionTrajectorysInsIdx() const;
    uint32_t GetTrafficLightsInsIdx() const;
    uint32_t GetEgoTrajectoryInsIdx() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode& config) override;

private:
    bool IsInsIdxValid() const;

    uint32_t locationInstanceIdx_;
    uint32_t navigationResultInstanceIdx_;
    uint32_t chassisReportInstanceIdx_;
    uint32_t egoTrajectoryInstanceIdx_;
    uint32_t fusionObjectArrayInstanceIdx_;
    uint32_t fusionLaneDetectionOutArrayInstanceIdx_;
    uint32_t predictionOutArrayInstanceIdx_;
    uint32_t tlDetectionOutArrayInstanceIdx_;

    std::unique_ptr<LocationReceiveBase> locationRec_;
    std::unique_ptr<NavigationResultReceiveBase> navigationResultRec_;
    std::unique_ptr<ChassisReportReceiveBase> chassisReportRec_;
    std::unique_ptr<FusionObjectReceiveBase> fusionObjectArrayRec_;
    std::unique_ptr<LanesReceiveBase> fusionLaneDetectionOutArrayRec_;
    std::unique_ptr<PredictionReceiveBase> predictionOutArrayRec_;
    std::unique_ptr<TrafficLightReceiveBase> tlDetectionOutArrayRec_;

    std::unique_ptr<EgoTrajectorySendBase> egoTrajectorySend_;
    std::vector<std::thread> threadPool_;
};
}  // namespace Adsfi
#endif  // HAF_ADSF_PLANNING_BASE_H
