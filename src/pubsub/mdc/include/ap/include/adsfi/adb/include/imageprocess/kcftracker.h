/*
+ * Description: 封装kcf
  * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
+ */
#ifndef KCF_TRACKER_H
#define KCF_TRACKER_H
#if defined(MDC_PRODUCTION_CORE)
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "opencv/haf_opencv.h"
#include "opencv/haf_cv_types.h"
#include "core/status.h"
#include "core/basic_types.h"
#include "core/types.h"
#include "core/logger.h"

namespace Adsfi {
struct RectPoint {
    int32_t ltx;
    int32_t lty;
    int32_t rbx;
    int32_t rby;
    RectPoint()
    {
        ltx = 0;
        lty = 0;
        rbx = 0;
        rby = 0;
    }
};

class HafKcfTracker {
public:
    HafKcfTracker();
    HafStatus Release();
    HafStatus Init(const ImageData &srcImage, const HafRect2f initRect);
    HafStatus Update(const ImageData &srcImage, HafRect2f &outRect);

private:
    void GetOutRect(const HafRect2i &inRect, const int32_t width, const int32_t height, HafCVRect &outRect) const;
    void GetGaussLabelF(HafMat &gaussLabelF, const HafCVSize &patternSz, const float32_t sigma) const;
    void GetHannWindow(HafMat &hannWindow, const HafCVSize &sz) const;
    void GetPatch(const HafMat &inImg, HafMat &outPatch, const HafRect2i &roiRect) const;
    HafStatus GetFeatures(const HafMat &img, HafMat &feat, const HafMat &hannWin);
    void FastFourierTrans2(HafMat &feat, HafMat &featSpectrum) const;
    void InvFastFourierTrans2(const HafMat &spectrum, HafMat &response) const;
    void GaussCorrelationKernel(HafMat &xF, HafMat &yF, HafMat &kernelF, const bool isTrain) const;
    float64_t CalcNorm(HafMat &src) const;
    void GetSubPixelPeak(const HafCVPoint &maxLoc, const HafMat &response, HafCVPoint2f &subPixLoc) const;
    void Train(HafMat &featSpectrum, HafMat &gaussLabelF, HafMat &alphaF) const;
    void Detect(HafMat &featSpectrum, HafMat &featModel, HafMat &alphaF, HafMat &response, HafCVPoint2f &pos) const;
    HafStatus CheckInputImage(const ImageData &srcImage) const;
    HafStatus CheckInputRect(const ImageData &srcImage, const HafRect2f &roi) const;
    HafStatus CheckIntersect(const ImageData &srcImage, HafCVRect &roi) const;
    void AdjustRoi(const ImageData &srcImage, HafCVRect &roi) const;
    void ImageConvert2Mat(const ImageData &src, HafMat &dst) const;
    void Rect2HafRect(const HafCVRect srcRect, Adsfi::HafRect2f &dstRect) const;
    void HafRect2Rect(const HafRect2f srcRect, HafCVRect &dstRect) const;
    void CvtImage(const HafMat &src, HafMat &dst) const;
    HafStatus PreUpdateCheck() const;

    int32_t transCellSz;
    int32_t transPatchNormSz;

    HafRect2i tgtBox;
    HafRect2i winBox;
    float32_t xZoom;
    float32_t yZoom;

    HafMat winPatch;
    HafMat transPatch;

    HafMat transGaussLabelF;
    HafMat transHannWin;
    HafMat transFeat;
    HafMat transModelF;
    HafMat transAlphaF;

    int32_t bWidth;
    int32_t bHeight;
    const float32_t fHalf = 0.5;
    const float32_t fTwo = 2.0;
    const int32_t widthIdx = 2;
    const int32_t dimensionSize = 3;
    const int32_t cellSize = 4;
    const int32_t marginSize = 5;
    const int32_t coeffSize = 6;
    const int32_t pointSize = 9;
    const int32_t featureBinSize = 9;
    const float32_t m2Pi = 2 * M_PI;
    bool isInit;
    bool isRelease;
};
}
#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
#endif // KCF_TRACKER_H
