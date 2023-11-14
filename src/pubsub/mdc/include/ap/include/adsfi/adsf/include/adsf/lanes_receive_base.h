/**
 * FUNCTION: Define LanesReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 **/

#ifndef HAF_ADSF_LANERECIEVE_BASE_H
#define HAF_ADSF_LANERECIEVE_BASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "core/types.h"
#include "object/lanes.h"
#include "mdc/adsfi/lanelinearrayinterface_proxy.h"

namespace Adsfi {
    class LanesReceiveBase : public DataReceiveBase<mdc::adsfi::proxy::LaneLineArrayInterfaceProxy,
        mdc::adsfi::proxy::LaneLineArrayInterfaceProxy::HandleType,
                            HafLaneDetectionOutArray> {
    public:
        explicit LanesReceiveBase(const uint32_t instanceIdxIn)
            : DataReceiveBase(instanceIdxIn) {};
        virtual ~LanesReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    };
}
#endif  // HAF_ADSF_LANERECIEVE_BASE_H
