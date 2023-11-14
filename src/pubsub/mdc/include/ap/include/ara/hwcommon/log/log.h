/*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
* Description: Log class header
* Create: 2019-7-2
*/
#ifndef INC_ARA_GODEL_COMMON_LOG_H
#define INC_ARA_GODEL_COMMON_LOG_H

#include <vector>
#include "ara/hwcommon/log/log_stream_buffer.h"
#include "ara/hwcommon/log/common.h"
#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <map>

namespace ara    {
namespace godel  {
namespace common {
namespace log    {
class BaseLog;
class Log;
class LogVariable {
public:
    static std::shared_ptr<LogVariable>& GetInstance();
    void SetAppId(const std::string& id);
    std::shared_ptr<Log> GetLogInstance(const std::string& ctxId, const std::string& ctxDescription);
    std::mutex logMutex;
    ~LogVariable() = default;
private:
    LogVariable() = default;
    std::unordered_map<std::string, std::shared_ptr<Log>> logModules;
    // The mutex is used in aos-core and the macro is not impl beacause of the head file is public for user.
    std::mutex mutex_;
    std::string appId_;
};

class Log {
public:
    struct LogLimitResult {
        bool  logLimit;
        std::uint32_t logLimitCount;
    };
    struct LogRecordInfo {
        std::uint64_t startTime;
        std::uint32_t printLogCount;
    };
    static std::shared_ptr<Log> getInstance(std::string const &ctxId, std::string const &ctxDescription);
    static std::shared_ptr<Log> GetLog(std::string const &ctxId);
    std::unordered_map<uint8_t, std::shared_ptr<BaseLog>> drivers {};
    Log(std::string const &ctxId, std::string const &ctxDescription);
    LogStreamBuffer fatal(std::string const &logUUID = "", std::uint32_t logNumber = 0) noexcept;
    LogStreamBuffer error(std::string const &logUUID = "", std::uint32_t logNumber = 0) noexcept;
    LogStreamBuffer warn(std::string const &logUUID = "", std::uint32_t logNumber = 0) noexcept;
    LogStreamBuffer info(std::string const &logUUID = "", std::uint32_t logNumber = 0) noexcept;
    LogStreamBuffer debug() noexcept;
    LogStreamBuffer verbose() noexcept;
    static void InitLogging(std::string const &appId,
        std::string const &appDescription,
        LogLevel appDefLogLevel = LogLevel::VRTF_COMMON_LOG_WARN,
        LogMode appLogMode = LogMode::LOG_REMOTE,
        std::string const &directoryPath = "");
    static void InitLog(std::string const &appId, std::string const &ctxId, std::string const &ctxDescription);
    void ParseDriverAccordingEnvVar(int32_t index);
    void SetLogLevel(LogLevel lv);
    LogLevel level;
    ~Log() {}
    std::string GetCtxId() const;
    bool IsValid() const;
private:
    static uint64_t GetCurrentMonotonicTime();
    void ParseDriverType();
    Log::LogLimitResult LimitLogPrintOfUnitTime(std::string  const &logUUID, std::uint32_t logNumber);
    friend class LogStreamBuffer;
    std::string ctxId_;
    std::string ctxDescription_;
    bool containsAraLog_;
    Log(Log const &other) = default;
    bool isValid_;

    std::map<size_t, LogRecordInfo> logLimit_;
    std::shared_ptr<LogVariable> logVariable_;
};
}
}
}
}
#endif
