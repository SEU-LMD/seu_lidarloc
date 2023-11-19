/**
 * Copyright (C) 2018. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DVPP_JPEG_H
#define DVPP_JPEG_H

#include <iostream>
#include <stdint.h>
#include "ExportMacro.h"

const int32_t GET_JPEGE_OUT_PARAMETER = 0;
const int32_t GET_JPEGD_OUT_PARAMETER = 1;
const int32_t GET_PNGD_OUT_PARAMETER  = 2;
const int32_t GET_JPEGD_TO_RGB_OUT_PARAMETER = 3;
constexpr uint32_t RGB_OUT_CHANNEL_NUM = 3;

typedef enum {
    JPEG_ERROR               = -1,
    JPEG_SUCCESS             = 0,
    JPEG_ERROR_INVALID_PARAM = 1,
    JPEG_ERROR_HARDWARE      = 2,
    JPEG_ERROR_RUNTIME       = 3
} JpegState;
// jpege supports the following format input
typedef enum {
    JPGENC_FORMAT_UYVY = 0x0,
    JPGENC_FORMAT_VYUY = 0x1,
    JPGENC_FORMAT_YVYU = 0x2,
    JPGENC_FORMAT_YUYV = 0x3,
    JPGENC_FORMAT_NV12 = 0x10,
    JPGENC_FORMAT_NV21 = 0x11
} eEncodeFormat;

// jpege input yuv format
typedef enum {
    JPGENC_FORMAT_YUV422 = 0x0,
    JPGENC_FORMAT_YUV420 = 0x10,
    JPGENC_FORMAT_BIT    = 0xF0
} eFormat;

// The application and release of the memory for hard decoding and soft decoding are inconsistent.
// The function object is encapsulated to mask the release details of the output memory.
// In addition, the function needs to be bound.
// The binary function and the one-meta-function are processed as non-parametric functions,
// this function is provided for the invoker to invoke as a callback function.
// usage: outBuf.cbFree = JpegCalBackFree( .. )
//        outBuf.cbFree()
class JpegCalBackFree {
public:
    JpegCalBackFree() : isMunmapUse_(false), addr4Free_(nullptr), siz4Free_(0) {}
    ~JpegCalBackFree() {}

    JpegCalBackFree(JpegCalBackFree& others);

    void setBuf(void* addr4Free);
    void setBuf(void* addr4Free, size_t siz4Free);

    void operator() ();
    const JpegCalBackFree& operator= (JpegCalBackFree& others);

private:
    bool isMunmapUse_;
    void* addr4Free_;
    size_t siz4Free_;
};


// jpeg encoder input data
typedef struct _sJpegeIn {
    eEncodeFormat format; // input YUV data format, which supports YUV422 packed and YUV420 Semi-planar.
    unsigned char* buf; // alloced with mmap with huge memory table, and aligned to 128.
    uint32_t bufSize; // ALIGN_UP( stride*height, 4096) for yuv422, ALIGN_UP( stride*height*3/2, 4096) for yuv420.
    uint32_t width; // input image width.
    uint32_t height; // input image height.
    uint32_t stride; // input image width.ALIGN_UP( width*2, 16) for yuv422, ALIGN_UP( width, 16) for yuv420.
    uint32_t heightAligned; // the height of the image after alignment.
    uint32_t level; // the qulitity of the jpg, 0~100, the larger the clearer img.
    _sJpegeIn()
    : format(JPGENC_FORMAT_UYVY), buf( nullptr ), bufSize(0),
    width(0), height(0), stride(0), heightAligned(0), level(100) {}
} sJpegeIn;


// jpeg encoder output data.
typedef struct _sJpegeOut {
    unsigned char* jpgData; // The starting address of the jpg data in the output buffer.
    uint32_t jpgSize; // Encoded jpg image data size.
    JpegCalBackFree cbFree; // callback function to free the output memory, when encode fail, no need to free.
    _sJpegeOut() : jpgData(nullptr), jpgSize(0), cbFree() {}
} sJpegeOut;


//jpegd param
enum jpegd_raw_format {
    DVPP_JPEG_DECODE_RAW_YUV_UNSUPPORT = -1,
    DVPP_JPEG_DECODE_RAW_YUV444 = 0,
    DVPP_JPEG_DECODE_RAW_YUV422_H2V1 = 1, // YUV422
    DVPP_JPEG_DECODE_RAW_YUV422_H1V2 = 2, // YUV440
    DVPP_JPEG_DECODE_RAW_YUV420 = 3,
    DVPP_JPEG_DECODE_RAW_YUV400 = 4,
    DVPP_JPEG_DECODE_RAW_MAX
};

enum jpegd_color_space{
    DVPP_JPEG_DECODE_OUT_UNKNOWN = -1,
    DVPP_JPEG_DECODE_OUT_YUV444 = 0, // Y/Cb/Cr (also known as YUV)
    DVPP_JPEG_DECODE_OUT_YUV422_H2V1 = 1, // YUV422
    DVPP_JPEG_DECODE_OUT_YUV422_H1V2 = 2, // YUV440
    DVPP_JPEG_DECODE_OUT_YUV420 = 3,
    DVPP_JPEG_DECODE_OUT_YUV400 = 4,
    DVPP_JPEG_DECODE_OUT_FORMAT_MAX
};

const uint32_t JPEGD_IN_BUFFER_SUFFIX = 8;

// jpegd in struct
struct JpegdIn {
    unsigned char* jpegData; // the input jpg data.
    uint32_t jpegDataSize; // the length of the input buf.
    bool isYUV420Need; // isYUV420Need == true, output yuv420 data, otherwize, raw format.
    bool isVBeforeU; // currently, only support V before U, reserved.
    uint32_t subDecodeZoneLeft;
    uint32_t subDecodeZoneRight;
    uint32_t subDecodeZoneTop;
    uint32_t subDecodeZoneBottom;

    JpegdIn()
        : jpegData(nullptr),
          jpegDataSize(0),
          isYUV420Need(true),
          isVBeforeU(true),
          subDecodeZoneLeft(0),
          subDecodeZoneRight(0),
          subDecodeZoneTop(0),
          subDecodeZoneBottom(0) {}
};

// jpegd out struct
struct JpegdOut {
    // Output yuv image data buf, the width and height of the image are the width and height after alignment.
    unsigned char* yuvData;
    uint32_t yuvDataSize;
    uint32_t imgWidth;
    uint32_t imgHeight;
    uint32_t imgWidthAligned; // Aligned width of the output image, align imgWidth up to 128.
    uint32_t imgHeightAligned; // Aligned height of the output image, align imgHeight up to 16.
    JpegCalBackFree cbFree; // Release the callback function of the output buf.
    enum jpegd_color_space outFormat; // Output yuv data format.

    JpegdOut() :
        yuvData(nullptr),
        yuvDataSize(0),
        imgWidth(0),
        imgHeight(0),
        imgWidthAligned(0),
        imgHeightAligned(0),
        cbFree(),
        outFormat( DVPP_JPEG_DECODE_OUT_UNKNOWN ) {}
};

// jpegd input data
struct jpegd_raw_data_info {
    unsigned char* jpeg_data;
    uint32_t jpeg_data_size; // 8 byte larger than jpeg file length, start address aligned to 128.
    enum jpegd_raw_format in_format; // The sampling format of yuv in the image.
    bool IsYUV420Need; // IsYUV420Need == true, user need yuv420 semi-planar output data, otherwize, do not need.
    bool isVBeforeU; // Currently, only support V before U, reserved.
    uint32_t subDecodeZoneLeft;
    uint32_t subDecodeZoneRight;
    uint32_t subDecodeZoneTop;
    uint32_t subDecodeZoneBottom;

    jpegd_raw_data_info()
        : jpeg_data(nullptr),
          jpeg_data_size(0),
          in_format(DVPP_JPEG_DECODE_RAW_YUV_UNSUPPORT),
          IsYUV420Need(true),
          isVBeforeU(true),
          subDecodeZoneLeft(0),
          subDecodeZoneRight(0),
          subDecodeZoneTop(0),
          subDecodeZoneBottom(0) {}
};

// jpegd output struct
struct jpegd_yuv_data_info {
    // Output buf of YUV picture data, the width and height of the picture are the width and height after alignment.
    unsigned char* yuv_data;
    uint32_t yuv_data_size; // The output data length is calculated from the width and height of the alignment.
    uint32_t img_width;
    uint32_t img_height;
    uint32_t img_width_aligned; // The width of the output image after aligned to 128.
    uint32_t img_height_aligned; // The height of the output image after aligned to 16.
    JpegCalBackFree cbFree; // Release the callback function of the output buf.
    enum jpegd_color_space out_format; // Output yuv data format.
    jpegd_yuv_data_info(): yuv_data(nullptr), yuv_data_size(0), img_width(0), \
        img_height(0), img_width_aligned(0), img_height_aligned(0), cbFree(), \
        out_format( DVPP_JPEG_DECODE_OUT_UNKNOWN )
    { }
};

struct JpegDecodeToRgbIn {
    uint8_t* jpegData;
    uint32_t jpegDataSize;
    uint32_t regionDecodeZoneLeft;
    uint32_t regionDecodeZoneRight;
    uint32_t regionDecodeZoneTop;
    uint32_t regionDecodeZoneBottom;

    JpegDecodeToRgbIn()
        : jpegData(nullptr),
          jpegDataSize(0),
          regionDecodeZoneLeft(0),
          regionDecodeZoneRight(0),
          regionDecodeZoneTop(0),
          regionDecodeZoneBottom(0) {}
};

struct JpegDecodeToRgbOut {
    uint8_t* rgbData; // only support RGB24
    uint32_t rgbDataSize;
    uint32_t outChannel;
    uint32_t imgWidth;
    uint32_t imgHeight;
    bool isHardwareDecode;

    JpegDecodeToRgbOut()
        : rgbData(nullptr),
          rgbDataSize(0),
          outChannel(RGB_OUT_CHANNEL_NUM),
          imgWidth(0),
          imgHeight(0),
          isHardwareDecode(true) {}
};

/*
* @brief : decode jpeg file to rgb
* @param [in] jpegDecodeToRgbIn : input data
* @param [out] jpegDecodeToRgbOut : output data
* @return : success, return DVPP_SUCCESS; fail, return DVPP_FAIL
*/
DVPP_EXPORT int32_t DvppJpegDecodeToRgb(JpegDecodeToRgbIn& jpegDecodeToRgbIn, JpegDecodeToRgbOut& jpegDecodeToRgbOut);

#endif // DVPP_JPEG_H