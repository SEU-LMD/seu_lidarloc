/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This class provides logging capability of the Ros-like interface.
 * Create: 2020-06-30
 */

#ifndef RTF_COM_UTILS_LOGGER_H_
#define RTF_COM_UTILS_LOGGER_H_

#include <memory>
#include "ara/hwcommon/log/log.h"

namespace rtf   {
namespace com   {
namespace utils {
class Logger {
public:
    using LogInterface = ara::godel::common::log::Log;
    using LogStream    = ara::godel::common::log::LogStreamBuffer;
    Logger();
    ~Logger() = default;
    static std::shared_ptr<Logger> GetInstance();
    LogStream Verbose() noexcept;
    LogStream Debug() noexcept;
    LogStream Info(bool changeLowLevel = false) noexcept;
    LogStream Warn(bool changeLowLevel = false) noexcept;
    LogStream Error(bool changeLowLevel = false) noexcept;
    LogStream Fatal() noexcept;
private:
    /* AXIVION enable style AutosarC++19_03-A7.1.3 */
    std::shared_ptr<LogInterface> logger_;
};
} // namespace utils
} // namespace com
} // namespace rtf
#endif // RTF_COM_UTILS_LOGGER_H_
