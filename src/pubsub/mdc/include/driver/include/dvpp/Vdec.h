/**
 * Copyright (C) 2018. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DVPP_VDEC_H
#define DVPP_VDEC_H

#include <cstddef>
#include <memory>
#include "DvppCommon.h"
#include "ExportMacro.h"

// vdec param
const int32_t vdec_default_width = 16;
const int32_t vdec_default_high = 16;
const int32_t vdec_default_frame = 30;

DVPP_EXPORT int32_t CreateVdecApi(IDVPPAPI*& pIDVPPAPI, int32_t singleton);
DVPP_EXPORT int32_t VdecCtl(IDVPPAPI*& pIDVPPAPI, int32_t CMD, dvppapi_ctl_msg* MSG, int32_t singleton);
DVPP_EXPORT int32_t DestroyVdecApi(IDVPPAPI*& pIDVPPAPI, int32_t singleton);
class IVDECAPI {};

class HIAI_DATA_SP
{
public:
    HIAI_DATA_SP()
    {
        frameIndex = 0;
        frameBuffer = NULL;
        frameSize = 0;
    }
    virtual ~HIAI_DATA_SP()
    {
    }
    void setFrameIndex(unsigned long long index)
    {
        frameIndex = index;
    }
    unsigned long long getFrameIndex()
    {
        return frameIndex;
    }
    void setFrameBuffer(void * frameBuff)
    {
        frameBuffer = frameBuff;
    }
    const void* getFrameBuffer()
    {
        return frameBuffer;
    }
    void setFrameSize(uint32_t size)
    {
        frameSize = size;
    }
    uint32_t getFrameSize()
    {
        return frameSize;
    }
private:
    unsigned long long frameIndex;
    void* frameBuffer;
    uint32_t frameSize;
    HIAI_DATA_SP(const HIAI_DATA_SP& );
    const HIAI_DATA_SP& operator= (const HIAI_DATA_SP&);
};

typedef struct FRAME
{
    int32_t height;
    int32_t width;
    int32_t realHeight;
    int32_t realWidth;
    unsigned char* buffer;
    int32_t buffer_size;
    uint32_t offset_payload_y;
    uint32_t offset_payload_c;
    uint32_t offset_head_y;
    uint32_t offset_head_c;
    uint32_t stride_payload;
    uint32_t stride_head;
    uint16_t bitdepth;
    char video_format[10];
    char image_format[10];
    //bool end_flag;
}FRAME;

enum ERRTYPE{
    //no err
    ERR_NONE = 0x0,
    //decoder state error
    ERR_INVALID_STATE = 0x10001,
    //hardware error
    ERR_HARDWARE,
    //scd decode fail
    ERR_SCD_CUT_FAIL,
    //vdm decode fail
    ERR_VDM_DECODE_FAIL,
    //alloc common memory fail
    ERR_ALLOC_MEM_FAIL,
    //Dynamic frame storage alloc fail
    ERR_ALLOC_DYNAMIC_MEM_FAIL,
    //config in and out port fail
    ERR_ALLOC_IN_OR_OUT_PORT_MEM_FAIL,
    //bit stream error
    ERR_BITSTREAM,
    //input video foramt error
    ERR_VIDEO_FORMAT,
    //output image foramt error
    ERR_IMAGE_FORMAT,
    //callback function is null error
    ERR_CALLBACK,
    //input buffer is null error
    ERR_INPUT_BUFFER,
    //input buffer size is zero error
    ERR_INBUF_SIZE,
    //create fill the buffer done thread failed
    ERR_THREAD_CREATE_FBD_FAIL,
    //create instance failed
    ERR_CREATE_INSTANCE_FAIL,
    //decoder init failed
    ERR_INIT_DECODER_FAIL,
    //get channel handle failed
    ERR_GET_CHANNEL_HANDLE_FAIL,
    //component set failed
    ERR_COMPONENT_SET_FAIL,
    //compare name failed
    ERR_COMPARE_NAME_FAIL,
    //other error
    ERR_OTHER,
    //decode no pic out(Interlaced scanning decode)
    ERR_DECODE_NOPIC = 0x20000
};

typedef struct VDECERR{
    ERRTYPE errType;
    uint16_t channelId;
}VDECERR;
// vdec input struct
struct vdec_in_msg {
    char video_format[10]; // Input video format,only support "h264" or "h265" default format is "h264".
    char image_format[10]; // Output frame format, the format is "nv21" or "nv12", the default is "nv12".
    void (*call_back)(FRAME* frame, void* hiai_data); // Caller callback function,FRAME is the output structure.
    char* in_buffer; // Input video stream memory, this stream is h264 or h265 bare stream.
    int32_t in_buffer_size; // Input video stream memory size.
    // After decoding, the parameter pointer of the result frame callback function is output,
    // and the pointer points to the specific structure defined by the caller.
    void * hiai_data;
    // The use of the hiai_data_sp smart pointer will mask hiai_data, hiai_data_sp and hiai_data can only be selected.
    std::shared_ptr<HIAI_DATA_SP> hiai_data_sp;
    int32_t channelId; // The ID of the decoding channel corresponding to the input code stream.
    void (*err_report)(VDECERR* vdecErr); // Error reporting callback function for notifying the user of an exception.
    bool isEOS; // Code stream end flag.
    int32_t isOneInOneOutMode; // Reserve, default value is 0.

    vdec_in_msg() {
        video_format[0] = 'h';
        video_format[1] = '2';
        video_format[2] = '6';
        video_format[3] = '4';
        video_format[4] = '\0';

        image_format[0] = 'n';
        image_format[1] = 'v';
        image_format[2] = '1';
        image_format[3] = '2';
        image_format[4] = '\0';

        in_buffer = nullptr;
        in_buffer_size = 0;
        call_back = nullptr;
        hiai_data = nullptr;
        hiai_data_sp = nullptr;
        channelId = 0;
        err_report = nullptr;
        isEOS = false;
        isOneInOneOutMode = 0;
    }
};

enum Status {
    FAIL = -1, // decode one frame unsuccessfully
    SUCC = 0, // deccode one frame successfully
    NOPIC = 1 //decode no pic out(Interlaced scanning decode)
};

enum VideoFormat {
    H264 = 0,
    HEVC = 1
};

enum ImageFormat {
    YUV420SP_NV12 = 0,
    YUV420SP_NV21 = 1
};

class FrameData {
public:
    FrameData();
    ~FrameData();
    void SetChanId(int32_t channelId);
    int32_t ChanId();
    void SetFrameId(uint64_t frameId);
    uint64_t FrameId();
    void SetStatus(enum Status status);
    enum Status Status();
    void SetAlignedWidth(int32_t alignedWidth);
    int32_t AlignedWidth();
    void SetAlignedHeight(int32_t alignedHeight);
    int32_t AlignedHeight();
    void SetRealWidth(int32_t realWidth);
    int32_t RealWidth();
    void SetRealHeight(int32_t realHeight);
    int32_t RealHeight();
    void SetOutBuffer(const uint8_t* outBuffer);
    const uint8_t* OutBuffer();
    void SetOutBufferSize(int32_t outBufferSize);
    int32_t OutBufferSize();
    void SetOffsetPayloadY(uint32_t offsetPayloadY);
    uint32_t OffsetPayloadY();
    void SetOffsetPayloadC(uint32_t offsetPayloadC);
    uint32_t OffsetPayloadC();
    void SetOffsetHeadY(uint32_t offsetHeadY);
    uint32_t OffsetHeadY();
    void SetOffsetHeadC(uint32_t offsetHeadC);
    uint32_t OffsetHeadC();
    void SetStridePayload(uint32_t stridePayload);
    uint32_t StridePayload();
    void SetStrideHead(uint32_t strideHead);
    uint32_t StrideHead();
    void SetBitdepth(uint32_t bitdepth);
    uint32_t Bitdepth();
    void SetVideoFormat(enum VideoFormat videoFormat);
    uint32_t VideoFormat();
    void SetImageFormat(enum ImageFormat imageFormat);
    uint32_t ImageFormat();
    void SetHiaiData(const void* hiaiData);
    const void* HiaiData();
    void SetCompressStatus(bool isCompressData);
    bool IsCompressData();
    void SetErrType(enum ERRTYPE errType);
    enum ERRTYPE ErrType();

private:
    FrameData(const FrameData&);
    const FrameData& operator= (const FrameData&);
    class FrameDataImpl;
    FrameDataImpl* frameDataImpl_;
};

typedef void (*FrameReturnFp)(FrameData& frameData);
typedef void (*ErrReportFp)(VDECERR* vdecErr);

class VdecInMsg {
public:
    VdecInMsg();
    ~VdecInMsg();
    void SetVideoFormat(enum VideoFormat videoFormat);
    enum VideoFormat VideoFormat();
    void SetImageFormat(enum ImageFormat imageFormat);
    enum ImageFormat ImageFormat();
    void SetInBuffer(const void* inBuffer);
    const void* InBuffer();
    void SetInBufferSize(int32_t inBufferSize);
    int32_t InBufferSize();
    void SetHiaiData(const void* hiaiData);
    const void* HiaiData();
    void SetHiaiDataSp(const std::shared_ptr<HIAI_DATA_SP> hiaiDataSp);
    std::shared_ptr<HIAI_DATA_SP> HiaiDataSp();
    void SetChannelId(int32_t channelId);
    int32_t ChannelId();
    void SetFrameReturn(FrameReturnFp frameReturnFp);
    const FrameReturnFp FrameReturn();
    void SetErrReport(ErrReportFp errReportFp);
    const ErrReportFp ErrReport();
    void SetEos(bool isEOS);
    bool IsEos();
    void SetOneInOneOutMode(bool isOneInOneOutMode);
    bool IsOneInOneOutMode();
    void SetChnlWidth(uint32_t chnlWidth);
    uint32_t ChnlWidth();
    void SetChnlHeight(uint32_t chnlHeight);
    uint32_t ChnlHeight();

private:
    VdecInMsg(const VdecInMsg&);
    const VdecInMsg& operator= (const VdecInMsg&);
    class VdecInMsgImpl;
    VdecInMsgImpl* vdecInMsgImpl_;
};

#endif // DVPP_VDEC_H
