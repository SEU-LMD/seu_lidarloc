/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: Support interface to sync log storage
 * Author: s00574258
 * Create: 2021-11-18
 */
#ifndef ARA_LOG_SYNC_H
#define ARA_LOG_SYNC_H

#include <cstdint>

namespace ara {
namespace log {
enum class LogSyncReturnType : uint8_t {
    SYNC_SUCCESS = 0x00U,
    SYNC_TIMEOUT = 0x01U,
    SYNC_FAILURE = 0x02U
};
LogSyncReturnType LogSync();
}  // namespace log
}  // namespace ara
#endif