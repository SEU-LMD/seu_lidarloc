/* *
 * FUNCTION: Define PredictionReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 *       */
#ifndef HAF_ADSF_PREDICTION_RECEIVE_BASE_H
#define HAF_ADSF_PREDICTION_RECEIVE_BASE_H

#include <shared_mutex>
#include "mdc/adsfi/trajetorypredictioninterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "object/prediction.h"
namespace Adsfi {
class PredictionReceiveBase : public DataReceiveBase<mdc::adsfi::proxy::TrajetoryPredictionInterfaceProxy,
    mdc::adsfi::proxy::TrajetoryPredictionInterfaceProxy::HandleType, HafObjPredictionOutArray> {
public:
    explicit PredictionReceiveBase(const uint32_t instanceIdx_) : DataReceiveBase(instanceIdx_) {};
    virtual ~PredictionReceiveBase();
    void RegisterHandle() override;
    void OnDataReceive();

private:
    void PreReceiveTimeTranslate(HafTime &toTime, const ara::adsfi::Time &fromTime) const;
    void PreReceivePointTranslate(Point3d &toPoint, const Point &fromPoint) const;
    void PreReceivePathPointTranslate(HafPathPoint &toPoint, const ara::adsfi::PathPoint &fromPoint) const;
    void PreReceiveTraPointTranslate(HafTrajectoryPointInPrediction &toPoint,
        const ara::adsfi::TrajectoryPoint &fromPoint) const;
    void PreReceivePerceptionObstacleTranslate(HafPerceptionObstacle &toObs,
        const ara::adsfi::PerceptionObstacle &fromObs) const;
    void PreReceiveObstacleFeatureVectorTranslate(HafObstacleFeature &toObs,
        const ara::adsfi::ObstacleFeature &fromObs) const;
    void PreReceiveObstacleFeatureTranslate(HafObstacleFeature &toObs,
        const ara::adsfi::ObstacleFeature &fromObs) const;
    void PreReceivePredictionObstacleTranslate(HafObjPredictionOutArray &toData,
        const ara::adsfi::ObjPredictionOutArray &fromData) const;
};
}
#endif
