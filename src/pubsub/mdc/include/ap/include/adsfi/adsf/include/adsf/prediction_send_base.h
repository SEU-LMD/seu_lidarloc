/* *
 * FUNCTION: Define Image Send Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 *     */
#ifndef HAF_ADSF_PREDICTION_SEND_BASE_H
#define HAF_ADSF_PREDICTION_SEND_BASE_H

#include <shared_mutex>
#include "mdc/adsfi/trajetorypredictioninterface_skeleton.h"
#include "adsf/data_send_base.h"
#include "core/types.h"
#include "object/prediction.h"

namespace Adsfi {
class PredictionSendBase
    : public DataSendBase<mdc::adsfi::skeleton::TrajetoryPredictionInterfaceSkeleton, HafObjPredictionOutArray> {
public:
    explicit PredictionSendBase(const uint32_t instanceIdxIn) : DataSendBase(instanceIdxIn) {};
    ~PredictionSendBase() override;
    void SendingData() override;

private:
    void PreSendTimeTranslate(ara::adsfi::Time &toTime, const HafTime &fromTime) const;
    void PreSendPointTranslate(Point &toPoint, const Point3d &fromPoint) const;
    void PreSendPathPointTranslate(ara::adsfi::PathPoint &toPoint, const HafPathPoint &fromPoint) const;
    void PreSendTraPointTranslate(ara::adsfi::TrajectoryPoint &toPoint,
        const HafTrajectoryPointInPrediction &fromPoint) const;
    void PreSendPerceptionObstacleTranslate(ara::adsfi::PerceptionObstacle &toObs,
        const HafPerceptionObstacle &fromObs) const;
    void PreSendObstacleFeatureVectorTranslate(ara::adsfi::ObstacleFeature &toObs,
        const HafObstacleFeature &fromObs) const;
    void PreSendObstacleFeatureTranslate(ara::adsfi::ObstacleFeature &toObs, const HafObstacleFeature &fromObs) const;
    void PreSendPredictionObstacleTranslate(ara::adsfi::ObjPredictionOutArray &toData,
        const HafObjPredictionOutArray &fromData) const;
};
}
#endif
