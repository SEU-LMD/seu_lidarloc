/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  SceneFeatureReceiveBase 用于接收FM故障管理的数据
 * Create: 2021-06-01
 */


#ifndef ADSF_FM_RECEIVE_BASE_H
#define ADSF_FM_RECEIVE_BASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "samm/samm_feature.h"
#include "mdc/fm/fmqueryservice_proxy.h"

namespace Adsfi {
    class FmReceiveBase : public DataReceiveBase<mdc::fm::proxy::FmQueryServiceProxy,
        mdc::fm::proxy::FmQueryServiceProxy::HandleType, std::vector<HafMdcFaultEvent>> {
    public:
        explicit FmReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID) {};
        void RegisterHandle() override;
        HafStatus DataReceive(std::vector<HafMdcFaultEvent>& samFmVec, int32_t timeOut);
        virtual ~FmReceiveBase();
    };
}
#endif // ADSF_FM_RECEIVE_BASE_H