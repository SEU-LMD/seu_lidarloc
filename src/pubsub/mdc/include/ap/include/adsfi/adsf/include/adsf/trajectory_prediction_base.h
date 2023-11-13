/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  TrajectoryPredictionBase.h 轨迹预测框架基类
 */

#ifndef HAF_ADSF_PREDICTION_BASE_H
#define HAF_ADSF_PREDICTION_BASE_H

#include <shared_mutex>
#include "adsfi_base.h"
#include "adsf/ego_trajectory_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/fusion_object_receive_base.h"
#include "adsf/prediction_send_base.h"
namespace Adsfi {
class TrajectoryPredictionBase : public AdsfiBase {
public:
    explicit TrajectoryPredictionBase(const std::string configFile) : AdsfiBase(configFile) {};
    ~TrajectoryPredictionBase() override;
    void Stop() override;
    HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
    HafStatus GetFusionObjects(std::shared_ptr<HafFusionOutArray<float32_t>> &data);
    HafStatus GetEgoTrajectory(std::shared_ptr<HafEgoTrajectory> &data);
    HafStatus SendResult(std::shared_ptr<HafObjPredictionOutArray> &data);

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    bool IsInsIdxValid() const;
    const uint32_t defaultLocationInstanceIdx_ = 113U;
    const uint32_t defaultObjectArrayInstanceIdx_ = 4001U;
    const uint32_t defaultEgoTrajectoryInstanceIdx_ = 103U;
    const uint32_t defaultPreTrajectoryInstanceIdx_ = 4681U;
    uint32_t locationInstanceIdx_ {defaultLocationInstanceIdx_};
    uint32_t objectArrayInstanceIdx_ {defaultObjectArrayInstanceIdx_};
    uint32_t egoTrajectoryInstanceIdx_ {defaultEgoTrajectoryInstanceIdx_};
    uint32_t preTrajectoryInstanceIdx_ {defaultPreTrajectoryInstanceIdx_};

    std::unique_ptr<LocationReceiveBase> locationRec_ {nullptr};
    std::unique_ptr<FusionObjectReceiveBase> objectArrayRec_ {nullptr};
    std::unique_ptr<EgoTrajectoryReceiveBase> egoTrajectoryRec_ {nullptr};
    std::unique_ptr<PredictionSendBase> preTrajectorySend_ {nullptr};

private:
    std::vector<std::thread> threadPool_;
};
}
#endif // HAF_ADSF_PREDICTION_BASE_H
