/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  InsReceiveBase.h 融合定位中Ins导远融合输出的接收头文件
 */

#ifndef HAF_ADSF_INS_RECEIVE_BASE_H
#define HAF_ADSF_INS_RECEIVE_BASE_H

#include <shared_mutex>
#include "core/types.h"
#include "data_receive_base.h"
#include "ara/ins/insinfoserviceinterface_proxy.h"
#include "location/gnss_info.h"
namespace Adsfi {
    class InsReceiveBase : public DataReceiveBase<ara::ins::proxy::InsInfoServiceInterfaceProxy,
        ara::ins::proxy::InsInfoServiceInterfaceProxy::HandleType, HafInsInfo> {
    public:
        explicit InsReceiveBase(const uint32_t instanceID):DataReceiveBase(instanceID){};
        template <typename T1, typename T2>
        static void GnssDataReceiveTranslate(const T1 &insSample, T2 &endData);
        template <typename T1, typename T2>
        static void InsDataReceiveTranslate(const T1 &insSample, T2 &endData);
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~InsReceiveBase();
    };
}
#endif  // HAF_ADSF_INS_RECEIVE_BASE_H
