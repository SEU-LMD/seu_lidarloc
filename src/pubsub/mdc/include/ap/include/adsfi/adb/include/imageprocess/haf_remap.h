/*
 * Description: 重映射接口
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 */
#ifndef HAF_IMAGE_PROCESS_HAF_REMAP_H
#define HAF_IMAGE_PROCESS_HAF_REMAP_H
#if defined(MDC_PRODUCTION_CORE)
#include "core/core.h"
#include "core/memory.h"
#include "image.h"
#include "remap.h"
#include "remap_wrap.h"
namespace Adsfi {
struct RemapRelationInfo {
    float32_t *remapData = nullptr;
    uint32_t width {};
    uint32_t height {};
    uint32_t dataSize {};
};

struct RemapParam {
    uint32_t srcWidth {};
    uint32_t srcHeight {};
    uint32_t dstWidth {};
    uint32_t dstHeight {};
    std::string remapOpNameID;
};

struct HafImageRemapHandle {
    aivector::RemapWrap *handle = nullptr;
    aivector::RemapAttrInfo attrInfo;
    aivector::RemapInputInfo inputInfo;
    aivector::RemapOutputInfo outputInfo;

    ~HafImageRemapHandle()
    {
        if (handle != nullptr) {
            delete handle;
            handle = nullptr;
        }
        HAF_LOG_INFO << "Finished running the destructor of the image remap handle.";
    }
};

HafStatus HafImageRemapInitialize(HafImageRemapHandle &remapHandle, const RemapRelationInfo &xMap,
    const RemapRelationInfo &yMap, const RemapParam &inputParam);
HafStatus HafImageRemapProcess(HafImageRemapHandle &remapHandle, const ImageData &inputImg, ImageData &outputImg,
    const HafStream stream = nullptr);
HafStatus HafImageRemapRelease(HafImageRemapHandle &remapHandle);
}
#endif // MDC_PRODUCTION_CORE || defined(MDC_PRODUCTION_CORE)
#endif
