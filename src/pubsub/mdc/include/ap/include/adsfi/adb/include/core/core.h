/* *
 * FUNCTION: Define Core Struct And Function
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *      */
#ifndef HAF_CORE_CORE_H__
#define HAF_CORE_CORE_H__

#include <string>
#include "core/status.h"
#include "core/types.h"
#include "core/basic_types.h"
#include "core/logger.h"
#include "acl/acl.h"

namespace Adsfi {
using HafEvent = aclrtEvent;
struct HafContext {
    aclrtContext aclContext{};
    std::string configPath{};
    int32_t deviceId{0};
};
enum class HafDeviceAttr {
    HAF_AICORE,
    HAF_VECTCORE
};

struct HafContextParameters {
    // If path is set to NULL, a current path ./ is used.
    // 配置文件中包括了算子包路径、芯片类型、编译服务器地址（可选，如果地址信息是locol）等信息，json格式。
    std::string configPath{};
    int32_t deviceId{0};  // 绑定的device ID
    HafDeviceAttr attr = HafDeviceAttr::HAF_AICORE;
};

HafStatus HafInitialize(HafContext& context, const HafContextParameters& contextParam);
HafStatus HafRelease(HafContext& context);
HafStatus HafCreateContext(HafContext& context);
HafStatus HafSetCurrentContext(HafContext& context);
HafStatus HafDestroyContext(HafContext& context);
HafStatus HafInitVectorDevice(const int32_t &deviceId);
HafStatus HafResetDevice(const int32_t &deviceId);
}  // namespace Adsfi

#endif  // HAF_CORE_CORE_H__
