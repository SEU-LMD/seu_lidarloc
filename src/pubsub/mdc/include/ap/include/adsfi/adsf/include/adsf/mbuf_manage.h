/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  mbuf manage class
 */
#ifndef HAF_ADSF_MBUF_MANAGE_H_
#define HAF_ADSF_MBUF_MANAGE_H_
#if defined(MDC_PRODUCTION_CORE)
#include "core/status.h"
#include "core/logger.h"
#include "driver/ascend_hal.h"
namespace Adsfi {
    class MbufManage {
    public:
        MbufManage (){};
        virtual ~MbufManage()
        {
            if (handle != nullptr) {
                (void)halBuffDeletePool(handle);
            }
        }
        HafStatus BufCreatePool(const uint32_t blkSize, const uint32_t blkNum);
        HafStatus CopyToMbuf(uintptr_t &mbuf, const uintptr_t &src, const uint32_t len) const;
    private:
        poolHandle handle = nullptr;
    };
}
#endif // MDC_PRODUCTION_CORE
#endif  // HAF_ADSF_MBUF_MANAGE_H_