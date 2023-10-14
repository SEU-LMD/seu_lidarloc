/*
+ * Description: 封装cropAndPaster的垂直拼接
+ * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
+ */
#ifndef HAF_IMAGEPROCESS_STITCH_H
#define HAF_IMAGEPROCESS_STITCH_H
#if defined(MDC_PRODUCTION_CORE)
#include <list>
#include "imageprocess/image.h"
#include "imageprocess/crop.h"

namespace Adsfi {
    /* ******************************************************************************
        函 数 名		:  HafImageStitch
        功能描述		:  封装HafImageCropAndPaste，将四张YUV420SPNV12图片垂直拼接成一张图片
        输入参数		:  HafChannelDesc的指针, 输入4张相同尺寸YUV420SPNV12图片队列，
                           输出1张垂直拼接后的YUV420SPNV12图片，HafStream
        输出参数		:  None 返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafImageStitch(const int32_t channelId, std::list<ImageData> &inputImageList,
        const ImageData &outputImage, const HafStream stream = nullptr);
}
#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
#endif // HAF_IMAGEPROCESS_STITCH_H