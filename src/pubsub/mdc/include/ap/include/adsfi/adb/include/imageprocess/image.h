/*
+ * Description: imageprocess的公共函数和结构体
  * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
+ */
#ifndef HAF_IMAGEPROCESS_IMAGE_H__
#define HAF_IMAGEPROCESS_IMAGE_H__
#include <limits>
#include "core/types.h"
#include "core/status.h"
#include "core/stream.h"
#include "acl/acl.h"
#if defined(MDC_PRODUCTION_CORE)
#include "hi_dvpp.h"
#elif defined(MDC_PRODUCTION_MDC300)
using HafChannel = void*;
#endif

namespace Adsfi {
#if defined(MDC_PRODUCTION_CORE)
/* ******************************************************************************
    函 数 名		:  HafCreateChannel
    功能描述		:  创建图像处理通道
    输入参数		:  图片处理通道号，取值范围：[0, 255]。
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafCreateChannel(int32_t &channelId);

/* ******************************************************************************
    函 数 名		:  HafDestroyChannel
    功能描述		:  销毁图像处理通道。
    输入参数		:  图片处理通道号，取值范围：[0, 255]。
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafDestroyChannel(int32_t &channelId);

#elif defined(MDC_PRODUCTION_MDC300)
/* ******************************************************************************
    函 数 名		:  HafCreateChannel
    功能描述		:  创建图像处理通道
    输入参数		:  图片处理通道指针。
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafCreateChannel(HafChannel &channel);

/* ******************************************************************************
    函 数 名		:  HafDestroyChannel
    功能描述		:  销毁图像处理通道。
    输入参数		:  图片处理通道指针。
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafDestroyChannel(HafChannel &channel);
#endif

/* ******************************************************************************
  函 数 名		:  HafGetNotAlignImage
  功能描述		:  复制dvpp内存中的对齐图片到非对齐的cpu内存中
  输入参数		:  输入图像
  输出参数		:  输出图像
  返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafGetNotAlignImage(const ImageData &srcImage, ImageData &dstImage);

/* ******************************************************************************
    函 数 名		:  HafImageMallocAligned
    功能描述		:  申请图像字节对齐内存。
    输入参数		:  输入图像。
    输出参数		:  申请图像字节对齐内存之后的图像
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafImageMallocAligned(ImageData &image);

/* ******************************************************************************
    函 数 名		:  HafImageMalloc
    功能描述		:  申请图像非字节对齐内存。
    输入参数		:  输入图像。
    输出参数		:  申请内存之后的图像
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafImageMalloc(ImageData &image);

/* ******************************************************************************
    函 数 名		:  HafImageDestroy
    功能描述		:  删除ImageData结构体中的*char_t
    输入参数		:  ImageData结构体
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafImageDestroy(ImageData &srcImage);

/* ******************************************************************************
    函 数 名		:  HafMemcpyToDvpp
    功能描述		:  将图片拷贝到DVPP内存
    输入参数		:  输入ImageData结构体
    输出参数		:  拷贝输出的ImageData结构体
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
HafStatus HafMemcpyToDvpp(const ImageData &srcImage, ImageData &dstImage);

/* ******************************************************************************
    函 数 名		:  GetNotAlignImage
    功能描述		:  得到非字节对齐的图像
    输入参数		:  ImageData结构体, 只支持DVPP buffer type.
    输出参数		:  非自己对齐的图像结构体
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
}
#endif
