/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI logger
 *              User should not use the header file directly.
 * Author: c00527270
 * Create: 2020-04-05
 */

#ifndef _PCFI_LOGGER_H_
#define _PCFI_LOGGER_H_

#include <thread>
#include <sstream>
#include "ara/log/logging.h"
#include "pcfi_base.h"

#define PCFI_LOG_HEAD GetThreaId() << "|" << SplitFilename(__FILE__) << ":" << \
    __LINE__ << " " << __FUNCTION__ << "() | "
#define PCFI_LOG_FATAL (GetLogger()->LogFatal() << PCFI_LOG_HEAD)
#define PCFI_LOG_ERROR (GetLogger()->LogError() << PCFI_LOG_HEAD)
#define PCFI_LOG_WARN (GetLogger()->LogWarn() << PCFI_LOG_HEAD)
#define PCFI_LOG_INFO (GetLogger()->LogInfo() << PCFI_LOG_HEAD)
#define PCFI_LOG_DEBUG (GetLogger()->LogDebug() << PCFI_LOG_HEAD)
#define PCFI_LOG_VERBOSE (GetLogger()->LogVerbose() << PCFI_LOG_HEAD)

namespace mdc {
namespace pcfi {
inline String SplitFilename(const String& str)
{
    const auto found = str.find_last_of("/");
    if (found == std::string::npos) {
        return str;
    }
    return str.substr(found + 1U);
}

inline ara::core::String GetThreaId()
{
    std::stringstream sinStr;
    const std::thread::id id = std::this_thread::get_id();
    sinStr << id;
    return sinStr.str();
}

class PcfiLogger {
public:
    static PcfiLogger& GetInstance()
    {
        static PcfiLogger instance;
        return instance;
    }

    ara::log::Logger* GetLogger() const
    {
        return m_logger;
    }

    void CreateLogger(const String& ctxId, const String& ctxDescription)
    {
        ara::core::StringView ctxIdView(ctxId.c_str());
        ara::core::StringView ctxDescView(ctxDescription.c_str());
        ara::log::Logger& logger = ara::log::CreateLogger(ctxIdView, ctxDescView);
        m_logger = &logger;
    }

private:
    PcfiLogger() = default;
    ~PcfiLogger() = default;
private:
    ara::log::Logger* m_logger;
};

inline ara::log::Logger* GetLogger()
{
    return PcfiLogger::GetInstance().GetLogger();
}
}
}
#endif
