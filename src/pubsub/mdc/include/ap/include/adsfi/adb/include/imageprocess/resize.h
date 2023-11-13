/*
+ * Description: 封装acl的acldvppVpcConvertColorAsync
  * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
+ */
#ifndef HAF_IMAGEPROCESS_RESIZE_H
#define HAF_IMAGEPROCESS_RESIZE_H
#include "imageprocess/image.h"
namespace Adsfi {
/* ******************************************************************************
    函 数 名		:  HafImageResize
    功能描述		:  封装acldvppVpcResizeAsync，实现图片缩放
    输入参数		:  HafChannelDesc的指针，输入图片的ImageData的引用，输出图片的ImageData的引用，插值方法，HafStream
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
****************************************************************************** */
#if defined(MDC_PRODUCTION_CORE)
HafStatus HafImageResize(const int32_t channelId, const ImageData &inputImage, ImageData &outputImage,
    const HafImageInterpolationType type, const int32_t timeout = -1);
// MDC_PRODUCTION_CORE || MDC_PRODUCTION_MDC210 || defined(MDC_PRODUCTION_CORE)
#elif defined(MDC_PRODUCTION_MDC300)
HafStatus HafImageResize(const HafChannel &channelId, const ImageData &inputImage, ImageData &outputImage,
    const HafImageInterpolationType type = HAF_INTER_LINEAR, const int32_t timeout = -1);
#endif // MDC_PRODUCTION_MDC300
}
#endif