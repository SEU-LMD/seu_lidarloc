/* *
 * FUNCTION: Define Status
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *    */
#ifndef HAF_STATUS_H__
#define HAF_STATUS_H__
namespace Adsfi {
    enum HafStatus {
        HAF_SUCCESS = 0,
        HAF_INVALID_ARGUMENT = 1,
        HAF_NULL = 2,
        HAF_NOT_READY = 3,
        HAF_INVALID_MEMCPY_DIRECTION = 4,
        HAF_OUT_OF_BOUNDS = 5,
        HAF_PROGRAM_STOPED = 6,
        HAF_TASK_TIMEOUT = 7,
        HAF_ERROR = 10,
        HAF_EVENT_ERROR = 11,
        HAF_MEMORY_ERROR = 12,
        HAF_STREAM_ERROR = 13
    };

    enum HafMempoolStatus {
        HAF_MEMPOOL_FREE = 0,
        HAF_MEMPOOL_READY = 1,
        HAF_MEMPOOL_BUSY = 2
    };

    enum class HafEventStatus {
        HAF_EVENT_STATUS_COMPLETE = 0,  // 完成
        HAF_EVENT_STATUS_NOT_READY = 1, // 未完成
        HAF_EVENT_STATUS_RESERVED = 2,  // 预留
    };
}
#endif // HAF_STATUS_H__
