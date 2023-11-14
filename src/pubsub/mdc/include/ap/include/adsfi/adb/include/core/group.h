/* *
 * FUNCTION: Define AI computing power group API
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2020. All rights reserved.
 *      */
#ifndef HAF_CORE_GROUP_H__
#define HAF_CORE_GROUP_H__
#if defined(MDC_PRODUCTION_CORE)
#include <string>
#include "core/status.h"
#include "core/types.h"
#include "core/basic_types.h"
#include "core/logger.h"
#include "acl/acl.h"

namespace Adsfi {
    enum class HafGroupAttr {
        HAF_GROUP_AICORE_INT = 0,  // 指定Group对应的aicore个数，属性值的数据类型为整型。
        HAF_GROUP_AIV_INT = 1, // 指定Group对应的vector core个数，属性值的数据类型为整型。
        HAF_GROUP_AIC_INT = 2,  // 指定Group对应的aicpu线程数，属性值的数据类型为整型。
        HAF_GROUP_SDMANUM_INT = 3,  // 内存异步拷贝的通道数，属性值的数据类型为整型。
        HAF_GROUP_ASQNUM_INT = 4,  // 指定Group下可以被同时调度执行的stream个数，小于或等于32。
        HAF_GROUP_GROUPID_INT = 5, // 指定Group的ID。属性值的数据类型为整型。
    };

    HafStatus HafSetGroup(const uint32_t groupID);
    HafStatus HafGetGroupCount(uint32_t &groupCount);
    HafStatus HafGetGroupInfoDetail(const uint32_t groupID,
                                    const HafGroupAttr groupAttr, void *attrValue,
                                    const size_t valueLen, size_t &paramRetSize);
}
#endif
#endif // HAF_CORE_GROUP_H__
