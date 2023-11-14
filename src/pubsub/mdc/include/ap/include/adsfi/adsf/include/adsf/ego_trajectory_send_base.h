/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  EgoTrajectorySendBase.h EgoTrajectory发送数据
 */

#ifndef HAF_ADSF_EGO_TRAJECTORY_SEND_BASE_H
#define HAF_ADSF_EGO_TRAJECTORY_SEND_BASE_H

#include <unistd.h>
#include <shared_mutex>
#include "ara/egotrajectory/egotrajectoryserviceinterface_skeleton.h"
#include "data_send_base.h"
#include "core/types.h"
#include "planning/ego_trajectory.h"

namespace Adsfi {
    class EgoTrajectorySendBase
        : public DataSendBase<ara::egotrajectory::skeleton::EgoTrajectoryServiceInterfaceSkeleton,
                 HafEgoTrajectory> {
    public:
        explicit EgoTrajectorySendBase(const uint32_t idx) : DataSendBase(idx){};
        ~EgoTrajectorySendBase() override;
        void SendingData() override;
    private:
        template <typename T1, typename T2>
        static void EgoSendHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void EgoSendEstopTransform(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void EgoSendWayPointTransform(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void AssignData(const T1& srcData, T2& desData);
    };
}
#endif // HAF_ADSF_EGO_TRAJECTORY_SEND_BASE_H
