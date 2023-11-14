/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  GnssReceiveBase.h 融合定位GPS的AP消息接受基类
 */

#ifndef HAF_ADSF_GNSS_RECEIVE_BASE_H
#define HAF_ADSF_GNSS_RECEIVE_BASE_H

#include <shared_mutex>
#include "ara/gnss/gnssinfoserviceinterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "location/gnss_info.h"

namespace Adsfi {
    class GnssReceiveBase : public DataReceiveBase<ara::gnss::proxy::GnssInfoServiceInterfaceProxy,
        ara::gnss::proxy::GnssInfoServiceInterfaceProxy::HandleType, HafGnssInfo> {
    public:
        explicit GnssReceiveBase(const uint32_t instanceID):DataReceiveBase(instanceID){};
        template <typename T1, typename T2>
        static void GnssReceiveBaseDataReceiveP1(const T1 &gnssSample, T2 &dataRcv);
        template <typename T1, typename T2>
        static void GnssReceiveBaseDataReceiveP2(const T1 &gnssSample, T2 &dataRcv);
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~GnssReceiveBase();
    };
}
#endif  // HAF_ADSF_GNSS_RECEIVE_BASE_H
