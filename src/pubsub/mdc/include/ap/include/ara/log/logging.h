/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Created by l00519163 on 2020/5/23.
 */

#ifndef LOGGING_H
#define LOGGING_H

#include "ara/log/common.h"
#include "ara/log/logger.h"
#include "ara/log/logstream.h"

namespace ara {
namespace log {
void InitLogging(std::string appId, std::string appDescription, LogLevel appDefLogLevel, LogMode logMode,
    std::string directoryPath = "") noexcept;

Logger &CreateLogger(ara::core::StringView ctxId, ara::core::StringView ctxDescription,
    LogLevel ctxDefLogLevel = LogLevel::kWarn) noexcept;

Logger &GetOperLogger() noexcept;

Logger &GetSecLogger() noexcept;

Logger &GetRunLogger() noexcept;
constexpr LogHex8 HexFormat(uint8_t value) noexcept
{
    return LogHex8 { value };
}

constexpr LogHex8 HexFormat(int8_t value) noexcept
{
    return LogHex8 { static_cast<uint8_t>(value) };
}

constexpr LogHex16 HexFormat(uint16_t value) noexcept
{
    return LogHex16 { value };
}

constexpr LogHex16 HexFormat(int16_t value) noexcept
{
    return LogHex16 { static_cast<uint16_t>(value) };
}

constexpr LogHex32 HexFormat(uint32_t value) noexcept
{
    return LogHex32 { value };
}

constexpr LogHex32 HexFormat(int32_t value) noexcept
{
    return LogHex32 { static_cast<uint32_t>(value) };
}

constexpr LogHex64 HexFormat(uint64_t value) noexcept
{
    return LogHex64 { value };
}

constexpr LogHex64 HexFormat(int64_t value) noexcept
{
    return LogHex64 { static_cast<uint64_t>(value) };
}

constexpr LogBin8 BinFormat(uint8_t value) noexcept
{
    return LogBin8 { value };
}

constexpr LogBin8 BinFormat(int8_t value) noexcept
{
    return LogBin8 { static_cast<uint8_t>(value) };
}

constexpr LogBin16 BinFormat(uint16_t value) noexcept
{
    return LogBin16 { value };
}

constexpr LogBin16 BinFormat(int16_t value) noexcept
{
    return LogBin16 { static_cast<uint16_t>(value) };
}


constexpr LogBin32 BinFormat(uint32_t value) noexcept
{
    return LogBin32 { value };
}

constexpr LogBin32 BinFormat(int32_t value) noexcept
{
    return LogBin32 { static_cast<uint32_t>(value) };
}

constexpr LogBin64 BinFormat(uint64_t value) noexcept
{
    return LogBin64 { value };
}

constexpr LogBin64 BinFormat(int64_t value) noexcept
{
    return LogBin64 { static_cast<uint64_t>(value) };
}

template<typename T> constexpr LogRawBuffer RawBuffer(const T &value) noexcept
{
    return LogRawBuffer { static_cast<const void *>(&value), sizeof(T) };
}
}
}
#endif