/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: camera data receive
 */
#ifndef HAF_ADSF_CAMERA_MBUF_RECIEVE_BASE_H
#define HAF_ADSF_CAMERA_MBUF_RECIEVE_BASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "core/types.h"

#include "mdc/cam/camera/cameradecodedmbufserviceinterface_proxy.h"
#include "mdc/cam/camera/cameradecodedserviceinterface_proxy.h"

#ifdef MDC_PRODUCTION_MDC300
using proxyType = mdc::cam::camera::proxy::CameraDecodedServiceInterfaceProxy;
using dataStruct = ara::camera::CameraPublishDecodedStruct;
#else
using proxyType = mdc::cam::camera::proxy::CameraDecodedMbufServiceInterfaceProxy;
using dataStruct = ara::camera::CameraDecodedMbufStruct;
#endif

namespace Adsfi {
class CameraReceiveBase : public DataReceiveBase<proxyType, proxyType::HandleType, ImageFrameV2> {
public:
    explicit CameraReceiveBase(const uint32_t instanceIdx, const int64_t time2live = 0, const size_t bufferSize = 5U)
        : DataReceiveBase(instanceIdx, time2live, bufferSize){};
    virtual ~CameraReceiveBase();
    /**
     * @brief 注册新数据到达时的回调函数，由AP调用。
     *
     * @return HafStatus
     */
    void RegisterHandle() override;
    /**
     * @brief 新数据到达时的回调函数。
     *
     */
    void OnDataReceive();

private:
    HafStatus ImageDataAssign(const dataStruct& sample);
};
};  // namespace Adsfi
#endif
