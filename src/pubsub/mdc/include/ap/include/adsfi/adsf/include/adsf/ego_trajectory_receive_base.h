/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  EgoTrajectoryReceiveBase.h 轨迹信息的AP消息接受
 */

#ifndef HAF_ADSF_EGO_TRAJECTORY_RECEIVEBASE_H
#define HAF_ADSF_EGO_TRAJECTORY_RECEIVEBASE_H

#include <shared_mutex>
#include "ara/egotrajectory/egotrajectoryserviceinterface_proxy.h"
#include "core/types.h"
#include "data_receive_base.h"
#include "planning/ego_trajectory.h"

namespace Adsfi {
    class EgoTrajectoryReceiveBase :
        public DataReceiveBase<ara::egotrajectory::proxy::EgoTrajectoryServiceInterfaceProxy,
        ara::egotrajectory::proxy::EgoTrajectoryServiceInterfaceProxy::HandleType, HafEgoTrajectory> {
    public:
        explicit EgoTrajectoryReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx){};
        virtual ~EgoTrajectoryReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    private:
        template <typename T1, typename T2>
        static void EgoReceiveHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void EgoReceiveEstopTranslate(const T1 &fromEstop, T2 &endEstop);
        template <typename T1, typename T2>
        static void EgoReceiveWayPointTranslate(const T1 &fromWayPoint, T2 &endWayPoint);
    };
}
#endif // HAF_ADSF_EGO_TRAJECTORY_RECEIVEBASE_H
