/* *
 * FUNCTION: Define Image Send Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
*  */
#ifndef HAF_ADSF_IMAGE_SEND_BASE_H
#define HAF_ADSF_IMAGE_SEND_BASE_H
#include <shared_mutex>
#include "adsf/data_send_base.h"
#include "core/types.h"
#include "object/object.h"
#include "mdc/cam/camera/cameradecodedmbufserviceinterface_skeleton.h"
#include "mdc/cam/camera/cameradecodedserviceinterface_skeleton.h"

#ifdef MDC_PRODUCTION_MDC300
using skeletonType = mdc::cam::camera::skeleton::CameraDecodedServiceInterfaceSkeleton;
using imgPdataStruct = ara::camera::CameraPublishDecodedStruct;
#else
#include "adsf/mbuf_manage.h"
using skeletonType = mdc::cam::camera::skeleton::CameraDecodedMbufServiceInterfaceSkeleton;
using imgPdataStruct = ara::camera::CameraDecodedMbufStruct;
#endif

namespace Adsfi {
    class ImageSendBase
        : public DataSendBase<skeletonType, ImageFrameV2> {
    public:
        explicit ImageSendBase(const uint32_t idx) : DataSendBase(idx) {};
        void SendingData() override;
#ifdef MDC_PRODUCTION_MDC300
#else
        int32_t Init(const uint32_t blkSize, const uint32_t blkNum);
#endif
        ~ImageSendBase() override;
    private:
        int32_t AssignData(imgPdataStruct &imagePub, std::shared_ptr<ImageFrameV2> &data) const;
#ifdef MDC_PRODUCTION_MDC300
#else
        uint32_t blkSize_ = 0U;
        std::unique_ptr<MbufManage> mbufManage_ = nullptr;
#endif
    };
}
#endif
