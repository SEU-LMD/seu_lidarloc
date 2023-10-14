/*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
* Description: LogStreamBuffer class header
* Create: 2019-7-2
*/
#ifndef INC_ARA_GODEL_COMMON_LOG_STREAM_BUFFER_H
#define INC_ARA_GODEL_COMMON_LOG_STREAM_BUFFER_H

#include <string>
#include "ara/hwcommon/log/common.h"
#include <memory>

namespace ara    {
namespace godel  {
namespace common {
namespace log    {
class Log;
class LogStreamBuffer {
public:
    LogStreamBuffer(LogLevel level, Log& logInstance, uint32_t logLimitCount = 0);
    ~LogStreamBuffer();
    LogStreamBuffer& operator<<(std::string const &str) noexcept;
    LogStreamBuffer& operator<<(uint8_t num) noexcept;
    LogStreamBuffer& operator<<(int8_t num) noexcept;
    LogStreamBuffer& operator<<(uint16_t num) noexcept;
    LogStreamBuffer& operator<<(int16_t num) noexcept;
    LogStreamBuffer& operator<<(uint32_t num) noexcept;
    LogStreamBuffer& operator<<(int32_t num) noexcept;
    LogStreamBuffer& operator<<(uint64_t num) noexcept;
    LogStreamBuffer& operator<<(int64_t num) noexcept;
    LogStreamBuffer& operator<<(float num) noexcept;
    LogStreamBuffer& operator<<(double num) noexcept;
    std::string GetMsg();
private:
    std::string msg;
    void printLog();
    bool DoStrSplice();
    LogLevel printLevel;
    Log& logInstance_;
    bool hasDoStrSplice_;
    bool canDoSplice_;
    const uint32_t logLimitCount_;
};
} // end log
} // end common
} // end godel
} // end ara
#endif
