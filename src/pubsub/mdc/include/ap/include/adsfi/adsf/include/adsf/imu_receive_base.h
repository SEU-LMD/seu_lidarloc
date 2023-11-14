/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  ImuReceiveBase.h 融合定位IMU的AP消息接受基类
 */

#ifndef HAF_ADSF_IMU_RECEIVE_BASE_H
#define HAF_ADSF_IMU_RECEIVE_BASE_H

#include <shared_mutex>
#include "ara/imu/imuinfoserviceinterface_proxy.h"
#include "core/types.h"
#include "data_receive_base.h"
#include "location/imu.h"
namespace Adsfi {
    class ImuReceiveBase
    : public DataReceiveBase<ara::imu::proxy::ImuInfoServiceInterfaceProxy,
            ara::imu::proxy::ImuInfoServiceInterfaceProxy::HandleType, HafIMU> {
    public:
        explicit ImuReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID){};
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~ImuReceiveBase();
    };
}
#endif  // HAF_ADSF_IMU_RECEIVE_BASE_H
