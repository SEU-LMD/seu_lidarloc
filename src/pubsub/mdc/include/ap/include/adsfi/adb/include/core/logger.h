/*
 * Description: AdsfiLogger
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */

#ifndef CORE_LOGGER_H
#define CORE_LOGGER_H

#include <thread>
#include <sstream>
#include "ara/log/logging.h"
namespace Adsfi {
    enum class HafLogModeType {
        HAF_REMOTE = 0,
        HAF_CONSOLE = 1,
        HAF_FILE = 2,
        HAF_REMOTE_CONSOLE = 3,
        HAF_REMOTE_FILE = 4,
        HAF_CONSOLE_FILE = 5,
        HAF_REMOTE_CONSOLE_FILE = 6
    };
    enum class HafLogLevelType {
        HAF_VERBOSE = 0,
        HAF_DEBUG = 1,
        HAF_INFO = 2,
        HAF_WARN = 3,
        HAF_ERROR = 4,
        HAF_FATAL = 5,
        HAF_OFF = 6
    };

    ara::log::LogMode ParseLogMode(const HafLogModeType logMode);
    ara::log::LogLevel ParseLogLevel(const HafLogLevelType logLevel);

    class AdsfiLogger {
    public:
        AdsfiLogger() : m_logger(nullptr) {};
        ~AdsfiLogger() = default;

        static AdsfiLogger &GetInstance()
        {
            static AdsfiLogger instance;
            return instance;
        }

        void InitLogging(const std::string appId, const std::string appDescription, const HafLogLevelType logLevel,
            const HafLogModeType logMode, const std::string directoryPath)
        {
            const ara::log::LogLevel level = ParseLogLevel(logLevel);
            const ara::log::LogMode mode = ParseLogMode(logMode);
            ara::log::InitLogging(appId, appDescription, level, mode, directoryPath);
        }

        void CreateLogger(const std::string ctxId, const std::string ctxDescription,
            const HafLogLevelType ctxLogLevel)
        {
            const ara::log::LogLevel level = ParseLogLevel(ctxLogLevel);
            ara::core::StringView ctxIdView(ctxId.c_str());
            ara::core::StringView ctxDescView(ctxDescription.c_str());
            ara::log::Logger &logger = ara::log::CreateLogger(ctxIdView, ctxDescView, level);
            m_logger = &logger;
        }

        ara::log::Logger *AdsfiGetLogger() const
        {
            return m_logger;
        }
    public:
        ara::log::Logger *m_logger;
    };

    inline ara::log::Logger *GetAdsfiLogger()
    {
        return AdsfiLogger::GetInstance().AdsfiGetLogger();
    }
}

#define HAF_LOG_HEAD  __FUNCTION__ << "() | "
#define HAF_LOG_FATAL (Adsfi::GetAdsfiLogger()->LogFatal() << HAF_LOG_HEAD)
#define HAF_LOG_ERROR (Adsfi::GetAdsfiLogger()->LogError() << HAF_LOG_HEAD)
#define HAF_LOG_WARN (Adsfi::GetAdsfiLogger()->LogWarn() << HAF_LOG_HEAD)
#define HAF_LOG_INFO (Adsfi::GetAdsfiLogger()->LogInfo() << HAF_LOG_HEAD)
#define HAF_LOG_DEBUG (Adsfi::GetAdsfiLogger()->LogDebug() << HAF_LOG_HEAD)
#define HAF_LOG_VERBOSE (Adsfi::GetAdsfiLogger()->LogVerbose() << HAF_LOG_HEAD)

#endif
