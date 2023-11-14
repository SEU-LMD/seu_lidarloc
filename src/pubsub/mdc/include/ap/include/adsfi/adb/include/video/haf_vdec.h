/* *
 * FUNCTION: Define H265 Video Decode Struct and Function
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *      */
#ifndef VDEC_H
#define VDEC_H
#if defined(MDC_PRODUCTION_CORE)
#include <mutex>
#include <vector>
#include "hi_dvpp.h"
#include "core/types.h"
#include "core/core.h"
#include "core/logger.h"

namespace Adsfi {
    struct VdecAttribute {
        uint32_t width;
        uint32_t height;
        uint32_t refFrameNum = 5U;       // [0,16]
        uint32_t displayFrameNum = 3U;   // [0,16]
        uint32_t frameBufCnt = 9U;       // refFrameNum plus displayFrameNum plus 1;
    };
    struct VdecStream {                  // hi_vdec_stream
        bool isEndOfStream = false;
        uint32_t streamLen = 0U;
        uint64_t timestamp = 0U;
        int32_t milliSec = 0;            // Timeout, -1 blocking mode; 0 unblocking mode; >0 timeOut mode
        uint8_t *streamAddr = nullptr;
        ~VdecStream()
        {
            if (streamAddr != nullptr) {
                free(streamAddr);
                streamAddr = nullptr;
            }
        };
    };
    struct VdecFrame {                   // hi_video_frame_info VIDEO_FRAME_S
        uint32_t frameBufferSize = 0U;
        uint32_t frameWidth = 0U;
        uint32_t frameHeight = 0U;
        uint32_t decFlag = 1U;        // Frame decode success flag, 0 = success; 1 = failded
        int32_t milliSec = 0;         // Timeout, -1 blocking mode; 0 unblocking mode; >0 timeOut mode
        uint8_t *frameAddr = nullptr;   // virtual address u64VirAddr
        ~VdecFrame()
        {
            if (frameAddr != nullptr) {
                free(frameAddr);
                frameAddr = nullptr;
            }
        };
    };

    class HafVdec {
    public:
        HafVdec(const uint32_t width, const uint32_t height)
            : vdecAttr_(), chnAttr_(), chnParam_(), stStream_(), outPicInfo_()
        {
            vdecAttr_.width = width;
            vdecAttr_.height = height;
        };
        HafStatus Init(const uint32_t refFrameNum, const uint32_t displayFrameNum, const int32_t inputChnId);
        HafStatus SendStream(const VdecStream &vStream);
        HafStatus GetFrame(VdecFrame &vFrame);
        uint32_t GetBufferSize() const;
        uint32_t GetWidth() const;
        uint32_t GetHeight() const;
        ~HafVdec();

    private:
        bool initStatus_ = false;
        HafStatus CreateChn();
        HafStatus GetChnAttr(hi_vdec_chn_attr &stAttr) const;
        HafStatus SetChn();
        HafStatus DestroyResource();
        HafStatus SetBlockStreamInput();
        HafStatus MallocBufferPool();
        HafStatus FreeBufferPool();
        HafStatus PutOutBufferToPool(hi_void *&outBuffer);
        HafStatus SetNormalStreamInput(hi_void *&outBuffer, const VdecStream &vStream);
        void ReleaseFrame(hi_void *&outputBuffer, const hi_video_frame_info &frameInfo, HafStatus &status);
        HafStatus SetSendExitStatus(const bool state);

        uint32_t outBufferSize_ = 0U;
        uint32_t outWidth_ = 0U;
        uint32_t outHeight_ = 0U;
        VdecAttribute vdecAttr_;
        hi_vdec_chn_attr chnAttr_;
        hi_vdec_chn_param chnParam_;

        hi_vdec_stream stStream_;       // SendStream
        hi_vdec_pic_info outPicInfo_;   // for SendStream

        const int32_t decChnIdMax_ = 47;
        const int32_t decChnIdMin_ = 25;
        // set video decode resolution and bufferSize;
        const uint32_t alignWidth_ = 16U;
        const uint32_t alignHeight_ = 2U;
        const uint32_t yuvMulFactor_ = 3U;
        const uint32_t yuvDivFactor_ = 2U;

        const uint32_t poolSize_ = 10U;
        std::vector<hi_void *> outBufferPool_;
        std::mutex outBufferPoolMtx_;
        bool sendExitStatus_ = false;

        hi_void *streamBuffer_ = nullptr;

        hi_vdec_chn chnId_ = 25;   // channel id
        const hi_payload_type enType_ = HI_PT_H265; // hi_payload_type of Dvpp API
        // hi_vdec_send_mode of Dvpp API:  HI_VDEC_SEND_MODE_FRAME = 1
        const hi_vdec_send_mode enMode_ = HI_VDEC_SEND_MODE_FRAME;
    };
}
#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
#endif // HAF_VENC_H
