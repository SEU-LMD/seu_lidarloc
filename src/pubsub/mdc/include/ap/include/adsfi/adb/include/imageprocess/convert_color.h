/*
+ * Description: 封装acl的acldvppVpcConvertColorAsync
  * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
+ */
#ifndef HAF_IMAGEPROCESS_CVTCOLOR_H
#define HAF_IMAGEPROCESS_CVTCOLOR_H
#include "imageprocess/image.h"
namespace Adsfi {
/* ******************************************************************************
    函 数 名		:  HafImageCvtColor
    功能描述		:  转换图片格式
    输入参数		:  channelId:VPC处理通道，inputImage:输入图片，timeout:超时时间
    输出参数		:  outputImage
    返回值      :  HAF_SUCCESS成功
****************************************************************************** */
#if defined(MDC_PRODUCTION_CORE)
HafStatus HafImageCvtColor(const int32_t channelId, const ImageData &inputImage, ImageData &outputImage,
    const int32_t timeout = -1);

#elif defined(MDC_PRODUCTION_MDC300)
HafStatus HafImageCvtColor(const HafChannel channelId, const ImageData &inputImage, ImageData &outputImage,
    const int32_t timeout = -1);

#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
}

#endif