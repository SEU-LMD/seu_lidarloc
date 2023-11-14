/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  SelectPointReceiveBase.h 终点选择数据接收
 */


#ifndef HAF_ADSF_SELECT_POINT_RECEIVE_BASE_H
#define HAF_ADSF_SELECT_POINT_RECEIVE_BASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "navigation/select_point.h"
#include "mdc/selectpoint/selectpointserviceinterface_proxy.h"
namespace Adsfi {
    class SelectPointReceiveBase : public DataReceiveBase<mdc::selectpoint::proxy::SelectPointServiceInterfaceProxy,
        mdc::selectpoint::proxy::SelectPointServiceInterfaceProxy::HandleType, HafSelectPoint> {
    public:
        explicit SelectPointReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID) {};
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~SelectPointReceiveBase();
    };
}
#endif // HAF_ADSF_SELECT_POINT_RECEIVE_BASE_H
