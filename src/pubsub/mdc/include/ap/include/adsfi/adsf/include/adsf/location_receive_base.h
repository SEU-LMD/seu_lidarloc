/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  LocationReceiveBase.h 定位数据接收
 */

#ifndef HAF_ADSF_LOCATION_RECEIVE_BASE_H
#define HAF_ADSF_LOCATION_RECEIVE_BASE_H

#include <shared_mutex>
#include "mdc/location/locationserviceinterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "location/location.h"

namespace Adsfi {
    class LocationReceiveBase : public DataReceiveBase<mdc::location::proxy::LocationServiceInterfaceProxy,
        mdc::location::proxy::LocationServiceInterfaceProxy::HandleType, HafLocation> {
    public:
        explicit LocationReceiveBase(const uint32_t instanceID)
            : DataReceiveBase(instanceID) {};
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~LocationReceiveBase();
    private:
        template<typename T1, typename T2> static void Point3dTransLate(const T1 &fromPoint, T2 &endPoint);
    };
}
#endif // HAF_ADSF_LOCATION_RECEIVE_BASE_H
