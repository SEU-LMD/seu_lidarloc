/**
 * FUNCTION: Define LanesSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 **/

#ifndef HAF_ADSF_LANESSEND_BASE_H
#define HAF_ADSF_LANESSEND_BASE_H

#include <shared_mutex>
#include "data_send_base.h"
#include "core/types.h"
#include "object/lanes.h"
#include "mdc/adsfi/lanelinearrayinterface_skeleton.h"
namespace Adsfi {
    class LanesSendBase
    : public DataSendBase<mdc::adsfi::skeleton::LaneLineArrayInterfaceSkeleton,
                                HafLaneDetectionOutArray> {
    public:
        explicit LanesSendBase(const uint32_t idx) : DataSendBase(idx){};
        ~LanesSendBase() override;
        void SendingData() override;
    };
}
#endif  //  HAF_ADSF_LANESSEND_BASE_H__
