/*
 * Description: 提供stream API接口
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */
#ifndef HAF_CORE_STREAM_H
#define HAF_CORE_STREAM_H

#include "core/core.h"

namespace Adsfi {
    using HafStream = aclrtStream;
    HafStatus HafStreamCreate(HafStream &stream);
    HafStatus HafStreamSynchronize(HafStream stream);
    HafStatus HafStreamDestroy(HafStream &stream);
}
#endif
