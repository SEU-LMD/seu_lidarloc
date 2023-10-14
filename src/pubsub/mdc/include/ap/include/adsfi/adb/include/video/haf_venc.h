/* *
 * FUNCTION: Define H265 Video Encode Struct and Function
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#ifndef HAF_VENC_H
#define HAF_VENC_H
#if defined(MDC_PRODUCTION_CORE)
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <map>
#include "core/types.h"
#include "core/logger.h"
#include "core/status.h"
#include "ascend_hal.h"
#include "hi_dvpp.h"

namespace Adsfi {
using VencStream = hi_venc_stream;
struct SampleBufferConfig { // hiSAMPLE_CAL_CONFIG_S
    hi_u32 u32VBSize;

    hi_u32 u32HeadStride;
    hi_u32 u32HeadSize;
    hi_u32 u32HeadYSize;

    hi_u32 u32MainStride;
    hi_u32 u32MainSize;
    hi_u32 u32MainYSize;

    hi_u32 u32ExtStride;
    hi_u32 u32ExtYSize;
};

class HafVenc {
public:
    HafVenc(const uint32_t width, const uint32_t height, const int32_t channelID)
        : width_(width), height_(height), channelID_(channelID) {};
    HafStatus Init();
    HafStatus SendFrame(const ImageData &srcImage);
    HafStatus GetStream(VencStream &pstStream);
    HafStatus ReleaseStream(VencStream &pstStream);
    ~HafVenc();

private:
    HafStatus CheckWidthHeightChannel() const;
    HafStatus CheckImage(const ImageData &srcImage) const;
    HafStatus CheckInit() const;
    HafStatus CreateChn();
    HafStatus StartRecvFrame();
    HafStatus SampleGetPicBufferConfig(SampleBufferConfig &pstCalConfig) const;
    HafStatus FillBuffer(hi_video_frame_info &videoFrame, const ImageData &srcImage);
    HafStatus DelBuffer(const hi_u64 packetTimeStamp);
    HafStatus SetFrameConfig();
    void SetVideoFrameInfo(hi_video_frame_info &videoFrame) const;
    HafStatus DestroyResource();
    hi_venc_mod_param modParam_;
    hi_venc_chn_attr chnAttr_;
    hi_venc_start_param recvParam_;
    SampleBufferConfig calConfig_;

    bool isInited_ = false;
    bool isStreamReady = false;
    // 总的可用通道范围为[0,46]，但[0,24]已分配给camera使用，adsfi使用通道范围为[25,46]
    const hi_s32 maxChannelID_ = 46;
    const hi_s32 minChannelID_ = 25;
    hi_s32 fd_ = 0;
    hi_u32 width_ = 1920U;
    hi_u32 height_ = 1080U;
    hi_u32 imgDataSize_ = 0U;
    hi_s32 channelID_ = 30;
    hi_u32 maxImgWidth_ = 4096U;
    hi_u32 maxImgHeight_ = 4096U;
    hi_u32 minWidthHeight_ = 128U;
    hi_u32 timeRef_ = 0U;
    hi_u32 maxLuminance_ = 1200U;
    hi_u32 minLuminance_ = 200U;
    hi_u32 bitWidth_ = 8U;
    hi_u32 alignSize_ = 16U;
    std::map<hi_u64, void *> frameDataTimeStampMap_;
    hi_u32 mapLimit_ = 15U;
    hi_u64 localTimeStamp_ = 0U;
    hi_pixel_format pixelFormat_ = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    hi_compress_mode cmpMode_ = HI_COMPRESS_MODE_NONE;
    const hi_u32 fps_ = 30U;
    const hi_u32 h265VbrMaxBitRate_ = 2000U;
};
}
#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
#endif // HAF_VENC_H
