/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Created by l00519163 on 2020/5/23.
 */

#ifndef LOGCOMMON_H
#define LOGCOMMON_H

#include <memory>
#include <iostream>

#include "securec.h"

namespace ara {
namespace log {
enum class LogMode : uint8_t {
    kRemote = 0x01,
    kFile = 0x02,
    kConsole = 0x04
};

enum class LogLevel : uint8_t {
    kOff = 0x00,
    kFatal = 0x01,
    kError = 0x02,
    kWarn = 0x03,
    kInfo = 0x04,
    kDebug = 0x05,
    kVerbose = 0x06
};

enum class LogReturnValue : int8_t {
    LOG_RETURN_LOGGING_DISABLE = -2,
    LOG_RETURN_ERROR = -1,
    LOG_RETURN_OK = 0,
    LOG_RETURN_TRUE = 1
};

inline std::ostream &operator <<(std::ostream &os, const LogLevel &level)
{
    switch (level) {
        case LogLevel::kOff:
            os << "off";
            break;
        case LogLevel::kFatal:
            os << "fatal";
            break;
        case LogLevel::kError:
            os << "error";
            break;
        case LogLevel::kWarn:
            os << "warn";
            break;
        case LogLevel::kInfo:
            os << "info";
            break;
        case LogLevel::kDebug:
            os << "debug";
            break;
        case LogLevel::kVerbose:
            os << "verbose";
            break;
        default:
            break;
    }
    return os;
}

inline LogMode operator|(const LogMode lhs, const LogMode rhs)
{
    return (static_cast<LogMode>(static_cast<typename std::underlying_type<LogMode>::type>(lhs) |
        static_cast<typename std::underlying_type<LogMode>::type>(rhs)));
}

inline LogMode operator&(const LogMode lhs, const LogMode rhs)
{
    return (static_cast<LogMode>(static_cast<typename std::underlying_type<LogMode>::type>(lhs) &
        static_cast<typename std::underlying_type<LogMode>::type>(rhs)));
}
}
}

#endif
