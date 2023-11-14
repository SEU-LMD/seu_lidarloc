/**
 * FUNCTION: Define ObjectReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 **/
#ifndef ADSF_LIDARRECEIVEBASE_H
#define ADSF_LIDARRECEIVEBASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "core/types.h"
#include "ara/lidar/lidarserviceinterface_proxy.h"
namespace Adsfi {
    class LidarReceiveBase
        : public DataReceiveBase<ara::lidar::proxy::LidarServiceInterfaceProxy,
                                ara::lidar::proxy::LidarServiceInterfaceProxy::HandleType, LidarFrame<PointXYZIRT>> {
    public:
        explicit LidarReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx) {};
        virtual ~LidarReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    };
}
#endif
