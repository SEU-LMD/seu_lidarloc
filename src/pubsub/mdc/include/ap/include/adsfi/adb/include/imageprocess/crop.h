/*
+ * Description: 封装VPC的CropResize和CropResizePaste
  * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
+ */
#ifndef HAF_IMAGEPROCESS_CROP_H
#define HAF_IMAGEPROCESS_CROP_H

#include "imageprocess/image.h"
namespace Adsfi {
#if defined(MDC_PRODUCTION_CORE)
/* **********************************************************************************************
    函 数 名		:  HafImageCrop
    功能描述		:  封装HI_MPI_VPC_CropResize，实现图片裁剪
    输入参数		:  通道号，输入图片，输出图片数组，裁剪区域数组，插值方法，HafStream
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
********************************************************************************************** */
HafStatus HafImageCrop(const int32_t channelId, const ImageData &inputImage, std::vector<ImageData> &outputImageVector,
    const std::vector<HafRoi> &cropAreaVector, const HafImageInterpolationType type, const int32_t timeout = -1);
/* **********************************************************************************************
    函 数 名		:  HafImageCropAndPaste
    功能描述		:  封装HI_MPI_VPC_CropResizePaste，实现图片裁剪粘贴
    输入参数		:  通道号，输入图片，输出图片数组，裁剪及粘贴区域数组，插值方法，HafStream
    输出参数		:  None
    返 回 值		:  0表示成功，其它值表示失败
********************************************************************************************** */
HafStatus HafImageCropAndPaste(const int32_t channelId, const ImageData &inputImage,
    std::vector<ImageData> &outputImageVector, const std::vector<HafRoi2> &cropPasteVector,
    const HafImageInterpolationType type, const int32_t timeout = -1);

#elif defined(MDC_PRODUCTION_MDC300)

HafStatus HafImageCrop(const HafChannel channelId, const ImageData &inputImage,
    std::vector<ImageData> &outputImageVector, const std::vector<HafRoi> &cropAreaVector,
    const HafImageInterpolationType type = HAF_INTER_LINEAR, const int32_t timeout = -1);

HafStatus HafImageCropAndPaste(const HafChannel channelId, const ImageData &inputImage,
    std::vector<ImageData> &outputImageVector, const std::vector<HafRoi2> &cropPasteVector,
    const HafImageInterpolationType type = HAF_INTER_LINEAR, const int32_t timeout = -1);

#endif // MDC_PRODUCTION_MDC300
}
#endif