/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  LocationSendBase.h 融合定位结果发送AP消息基类
 */

#ifndef HAF_ADSF_FUSION_LOCATION_RESULT_SEND_BASE_H
#define HAF_ADSF_FUSION_LOCATION_RESULT_SEND_BASE_H

#include <shared_mutex>
#include "data_send_base.h"
#include "core/types.h"
#include "location/location.h"
#include "mdc/location/locationserviceinterface_skeleton.h"
namespace Adsfi {
    class LocationSendBase
    : public DataSendBase<mdc::location::skeleton::LocationServiceInterfaceSkeleton,
                            HafLocation> {
    public:
        explicit LocationSendBase(const uint32_t instanceIdxIn):DataSendBase(instanceIdxIn){};
        virtual ~LocationSendBase();
        void SendingData() override;
    private:
        template<typename T1, typename T2> static void Point3dTransLate(const T1 &fromPoint, T2 &endPoint);
        template<typename T1, typename T2> static void RawDataTransLate(const T1 &source, T2 &des);
    };
}
#endif  // HAF_ADSF_FUSION_LOCATION_RESULT_SEND_BASE_H
