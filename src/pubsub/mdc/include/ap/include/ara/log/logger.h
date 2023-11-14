/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Created by l00519163 on 2020/5/23.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <atomic>
#include <memory>

#include "ara/log/common.h"
#include "ara/core/string_view.h"
#include "ara/core/string.h"


namespace ara {
namespace log {
class LogStream;
class Logger final {
public:
    Logger(ara::core::String ctxId, ara::core::String ctxDescription, ara::log::LogLevel ctxDefLogLevel);

    ~Logger();

    LogStream LogFatal() noexcept;

    LogStream LogError() noexcept;

    LogStream LogWarn() noexcept;

    LogStream LogInfo() noexcept;

    LogStream LogDebug() noexcept;

    LogStream LogVerbose() noexcept;

    bool IsEnabled(LogLevel logLevel) const noexcept;

    bool SetLogLevel(const LogLevel logLevel) const noexcept;

    LogLevel GetLogLevel() const noexcept;

    const ara::core::String& GetId() const noexcept;

    const ara::core::String& GetContextDescription() const noexcept;

    uint8_t GetSeqNum() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};
}
}

#endif
