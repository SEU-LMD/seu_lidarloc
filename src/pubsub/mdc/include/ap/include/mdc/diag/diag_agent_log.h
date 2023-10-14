/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 诊断管理客户端日志打印类(deprecated)
 * Create: 2020-03-04
 */
#ifndef DIAG_AGENT_LOG_H
#define DIAG_AGENT_LOG_H
#include <string>
#include <functional>

namespace mdc {
namespace diag {
enum class DiagAgentLogLevel :uint8_t {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
};
/* deprecated */
class DiagAgentLog {
public:
    using DiagAgentLogHandler = std::function<void(DiagAgentLogLevel, const std::string&)>;

    static DiagAgentLog& GetInstance();

    /* deprecated */
    void SetLogHanlder(const DiagAgentLogHandler logHandler) const;
};
}
}
#endif
